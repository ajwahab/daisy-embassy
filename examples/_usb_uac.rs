#![no_std]
#![no_main]

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use daisy_embassy::audio::{Idle, Interface};
use daisy_embassy::led::UserLed;
use defmt::{panic, *};
use embassy_executor::InterruptExecutor;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::interrupt::Priority;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt, pac, peripherals, timer, usb};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, TimeoutError, Timer, WithTimeout};
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use embassy_usb::driver::EndpointError;
use micromath::F32Ext;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

#[interrupt]
unsafe fn UART5() {
    unsafe { AUDIO_EXECUTOR.on_interrupt() }
}

static AUDIO_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

static TIMER: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<timer::low_level::Timer<peripherals::TIM2>>>,
> = Mutex::new(RefCell::new(None));

// A counter signal that is written by the feedback timer, once every `FEEDBACK_REFRESH_PERIOD`.
// At that point, a feedback value is sent to the host.
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

// Stereo input
pub const INPUT_CHANNEL_COUNT: usize = 2;

// This example uses a fixed sample rate of 48 kHz.
pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 120_000_000;

// Use 32 bit samples, which allow for a lot of (software) volume adjustment without degradation of quality.
pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width4Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

// Size of audio samples per 1 ms - for the full-speed USB frame period of 1 ms.
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_S.div_ceil(1000);

// Select front left and right audio channels.
pub const AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] =
    [uac1::Channel::LeftFront, uac1::Channel::RightFront];

// Factor of two as a margin for feedback (this is an excessive amount)
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// Feedback is provided in 10.14 format for full-speed endpoints.
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period8Frames;
const FEEDBACK_SHIFT: usize = 14;

const TICKS_PER_SAMPLE: f32 = (FEEDBACK_COUNTER_TICK_RATE as f32) / (SAMPLE_RATE_HZ as f32);

// Feedback value should not more more than 1 from the ideal value
const FEEDBACK_LIMIT_LOWER: f32 = ((((SAMPLE_RATE_HZ / 1000) - 1) << FEEDBACK_SHIFT) + 1) as f32;
const FEEDBACK_LIMIT_UPPER: f32 = ((((SAMPLE_RATE_HZ / 1000) + 1) << FEEDBACK_SHIFT) - 1) as f32;

const AUDIO_BUFF_SIZE: usize = USB_MAX_SAMPLE_COUNT * 2; //4 packets, 384 samples

const TIMER_CHANNEL: timer::Channel = timer::Channel::Ch1;

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

impl From<TimeoutError> for Disconnected {
    fn from(_val: TimeoutError) -> Self {
        Self {}
    }
}

// Model values
static STREAM_ACTIVE: AtomicBool = AtomicBool::new(false);
static GAIN_L: AtomicI32 = AtomicI32::new(0);
static GAIN_R: AtomicI32 = AtomicI32::new(0);

// Reporting values
static FEEDBACK_VALUE: AtomicU32 = AtomicU32::new(0);
static FEEDBACK_COUNT: AtomicU32 = AtomicU32::new(0);
static SAMPLES_RECEIVED: AtomicU32 = AtomicU32::new(0);
static SAMPLES_CONSUMED: AtomicU32 = AtomicU32::new(0);
static SAMPLES_MISSED: AtomicU32 = AtomicU32::new(0);

/// Sends feedback messages to the host.
///
/// The `feedback_factor` scales the timer's counter value so that the result is the number of samples that this device
/// played back during one SOF period (1 ms) - in 10.14 format. This assumes that the playback peripheral (e.g. SAI)
/// is clocked by the same source as the timer that counts the feedback value.
async fn feedback_handler<'d, T: usb::Instance + 'd>(
    feedback: &mut speaker::Feedback<'d, usb::Driver<'d, T>>,
    feedback_factor: f32,
) -> Result<(), Disconnected> {
    // Number of samples processed per frame, based on the counter values.
    // Set to the ideal value initially
    let mut samples_per_frame: f32 = (((SAMPLE_RATE_HZ) << FEEDBACK_SHIFT) / 1000) as f32;

    // Remainder from rounding the feedback value, added on to the next value so as not to lose it
    let mut remainder = 0.0f32;

    info!("feedback handler started");
    // Clear any existing FB value to make sure we don't start on the wrong frame
    let _ = FEEDBACK_SIGNAL.try_take();
    loop {
        let counter = FEEDBACK_SIGNAL.wait().await;

        let new_samples_per_frame = (counter as f32 * feedback_factor)
            .clamp(FEEDBACK_LIMIT_LOWER as f32, FEEDBACK_LIMIT_UPPER as f32);

        samples_per_frame += (new_samples_per_frame - samples_per_frame) * 0.1;

        let feedback_value_exact = samples_per_frame + remainder;
        let feedback_value_rounded = feedback_value_exact.floor();
        remainder = feedback_value_exact - feedback_value_rounded;
        let feedback_value_q14 = feedback_value_rounded as u32;
        FEEDBACK_VALUE.store(feedback_value_q14, Ordering::Relaxed);

        // The timeout is necessary to prevent the packet from being queued up during the frame after the
        // previous one was collected - and getting the wrong EONUM value
        match feedback
            .write_packet(&feedback_value_q14.to_le_bytes()[..3])
            .with_timeout(Duration::from_micros(10))
            .await
        {
            Ok(res) => {
                let r = res?;
                FEEDBACK_COUNT.fetch_add(1, Ordering::Relaxed);
                r
            }
            // timeout - ignore
            Err(_) => {}
        };
    }
}

/// Handles streaming of audio data from the host.
async fn stream_handler<'d, T: usb::Instance + 'd>(
    stream: &mut speaker::Stream<'d, usb::Driver<'d, T>>,
    sender: &mut channel::Sender<'static, CriticalSectionRawMutex, i32, AUDIO_BUFF_SIZE>,
) -> Result<(), Disconnected> {
    SAMPLES_RECEIVED.store(0, Ordering::Relaxed);
    SAMPLES_CONSUMED.store(0, Ordering::Relaxed);
    info!("stream handler started");

    loop {
        let mut usb_data = [0u8; USB_MAX_PACKET_SIZE];
        let data_size = stream
            .read_packet(&mut usb_data)
            .with_timeout(Duration::from_millis(2))
            .await??;

        let word_count = data_size / SAMPLE_SIZE;

        SAMPLES_RECEIVED.fetch_add(word_count as u32, Ordering::Relaxed);

        for sample in usb_data
            .as_chunks::<SAMPLE_SIZE>()
            .0
            .into_iter()
            .map(|s| i32::from_le_bytes(*s))
            .take(word_count)
        {
            sender.send(sample).await;
        }
        STREAM_ACTIVE.store(true, Ordering::Relaxed);
    }
}

/// Receives audio samples from the USB streaming task and can play them back.
#[embassy_executor::task]
async fn audio_receiver_task(
    interface: Interface<'static, Idle>,
    receiver: channel::Receiver<'static, CriticalSectionRawMutex, i32, AUDIO_BUFF_SIZE>,
    mut led: UserLed<'static>,
) {
    let mut interface = unwrap!(interface.start_interface().await);
    let mut playing = false;

    loop {
        let Err(e) = interface
            .start_callback(|_input, output| {
                let stream_active = STREAM_ACTIVE.load(Ordering::Relaxed);
                match (playing, stream_active) {
                    // Start condition - start playing as soon as buffer is sufficiantly full
                    (false, true) => playing = receiver.len() > AUDIO_BUFF_SIZE / 2,
                    // Stop condition - run out buffer then stop
                    (true, false) => playing = receiver.len() > 0,
                    _ => {}
                }

                if !playing {
                    output.fill(0);
                    return;
                }

                let gain_l = GAIN_L.load(Ordering::Relaxed) as i64;
                let gain_r = GAIN_R.load(Ordering::Relaxed) as i64;

                led.off();
                for (i, os) in output.iter_mut().enumerate() {
                    let sample = receiver.try_receive().unwrap_or_else(|_| {
                        if stream_active {
                            // Buffer Underrun!
                            led.on();
                            SAMPLES_MISSED.fetch_add(1, Ordering::Relaxed);
                        }
                        Default::default()
                    });
                    // Gain is q23, further 8 bit shift converts from 32 bit to 24 bit
                    *os = match i % 2 {
                        0 => (sample as i64 * gain_l) >> (23 + 8),
                        _ => (sample as i64 * gain_r) >> (23 + 8),
                    } as u32;
                }
                SAMPLES_CONSUMED.fetch_add(output.len() as u32, Ordering::Relaxed);
            })
            .await;

        error!("Audio Error {}", e);
    }
}

/// Receives audio samples from the host.
#[embassy_executor::task]
async fn usb_streaming_task(
    mut stream: speaker::Stream<'static, usb::Driver<'static, peripherals::USB_OTG_FS>>,
    mut sender: channel::Sender<'static, CriticalSectionRawMutex, i32, AUDIO_BUFF_SIZE>,
) {
    loop {
        stream.wait_connection().await;
        _ = stream_handler(&mut stream, &mut sender).await;
        STREAM_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Sends sample rate feedback to the host.
#[embassy_executor::task]
async fn usb_feedback_task(
    mut feedback: speaker::Feedback<'static, usb::Driver<'static, peripherals::USB_OTG_FS>>,
) {
    let feedback_factor = (1 << FEEDBACK_SHIFT) as f32 / TICKS_PER_SAMPLE;
    info!("Using a feedback factor of {}.", feedback_factor);

    loop {
        feedback.wait_connection().await;
        _ = feedback_handler(&mut feedback, feedback_factor).await;
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut usb_device: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB_OTG_FS>>,
) {
    usb_device.run().await;
}

/// Checks for changes on the control monitor of the class.
///
/// In this case, monitor changes of volume or mute state.
#[embassy_executor::task]
async fn usb_control_task(control_monitor: speaker::ControlMonitor<'static>) {
    loop {
        control_monitor.changed().await;

        for channel in AUDIO_CHANNELS {
            let volume = control_monitor.volume(channel).unwrap();
            info!("Volume changed to {} on channel {}.", volume, channel);

            let gain = match volume {
                speaker::Volume::Muted => 0.0,
                speaker::Volume::DeciBel(db) => 10.0.powf(db / 20.0) * 8_388_608.0 + 0.5,
            } as i32;

            match channel {
                uac1::Channel::LeftFront => GAIN_L.store(gain, Ordering::Relaxed),
                uac1::Channel::RightFront => GAIN_R.store(gain, Ordering::Relaxed),
                _ => {}
            }
        }
    }
}

/// Feedback value measurement and calculation
///
/// Used for measuring/calculating the number of samples that were received from the host during the
/// `FEEDBACK_REFRESH_PERIOD`.
///
/// Configured in this example with
/// - a refresh period of 8 ms, and
/// - a tick rate of 48 MHz.
///
/// This gives an (ideal) counter value of 384,000 for every update of the `FEEDBACK_SIGNAL`.
///
/// In this application, the timer is clocked by an internal clock source. A popular choice is to clock the timer from
/// the MCLK output of the SAI peripheral, which allows the SAI peripheral to use an external clock. However, this
/// requires wiring the MCLK output to the timer clock input.
#[interrupt]
fn TIM2() {
    // Count up frames and emit a signal, when the refresh period is reached.
    let regs = pac::USB_OTG_FS;
    critical_section::with(|cs| {
        let mut guard = TIMER.borrow(cs).borrow_mut();
        let timer = guard.as_mut().unwrap();
        if timer.get_input_interrupt(TIMER_CHANNEL) {
            let frame_number = regs.dsts().read().fnsof();
            // Send the signal one frame before the feedback will be requested
            if (frame_number + 1) % FEEDBACK_REFRESH_PERIOD.frame_count() as u16 == 0 {
                let ticks = timer.get_capture_value(TIMER_CHANNEL);
                FEEDBACK_SIGNAL.signal(ticks);
            }
            timer.clear_input_interrupt(TIMER_CHANNEL);
        }
    });
}

// If you are trying this and your USB device doesn't connect, the most
// common issues are the RCC config and vbus_detection
//
// See https://embassy.dev/book/#_the_usb_examples_are_not_working_on_my_board_is_there_anything_else_i_need_to_configure
// for more information.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");
    let config = daisy_embassy::default_rcc();
    let p = embassy_stm32::init(config);
    let board = daisy_embassy::new_daisy_board!(p);

    let interface = board
        .audio_peripherals
        .prepare_interface(Default::default())
        .await;

    // Configure all required buffers in a static way.
    debug!("USB packet size is {} byte", USB_MAX_PACKET_SIZE);
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 32]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 32]);

    const CONTROL_BUF_SIZE: usize = 64;
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; CONTROL_BUF_SIZE]);

    const FEEDBACK_BUF_SIZE: usize = 4;
    static EP_OUT_BUFFER: StaticCell<
        [u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE],
    > = StaticCell::new();
    let ep_out_buffer =
        EP_OUT_BUFFER.init([0u8; FEEDBACK_BUF_SIZE + CONTROL_BUF_SIZE + USB_MAX_PACKET_SIZE]);

    static STATE: StaticCell<speaker::State> = StaticCell::new();
    let state = STATE.init(speaker::State::new());

    // Create the driver, from the HAL.
    let mut usb_config = usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    usb_config.vbus_detection = false;

    let usb_driver = usb::Driver::new_fs(
        board.usb_peripherals.usb_otg_fs,
        Irqs,
        board.usb_peripherals.pins.DP,
        board.usb_peripherals.pins.DN,
        ep_out_buffer,
        usb_config,
    );

    // Basic USB device configuration
    let mut config = embassy_usb::Config::new(0xdead, 0xbeef);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-audio-speaker example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create the UAC1 Speaker class components
    let (stream, feedback, control_monitor) = Speaker::new(
        &mut builder,
        state,
        USB_MAX_PACKET_SIZE as u16,
        SAMPLE_WIDTH,
        &[SAMPLE_RATE_HZ],
        &AUDIO_CHANNELS,
        FEEDBACK_REFRESH_PERIOD,
    );

    // Create the USB device
    let usb_device = builder.build();

    static AUDIO_CHANNEL: StaticCell<
        channel::Channel<CriticalSectionRawMutex, i32, AUDIO_BUFF_SIZE>,
    > = StaticCell::new();
    let channel = AUDIO_CHANNEL.init(channel::Channel::new());
    let sender = channel.sender();
    let receiver = channel.receiver();

    // Run a timer for counting between SOF interrupts.
    let mut tim2 = timer::low_level::Timer::new(p.TIM2);
    tim2.set_tick_freq(Hertz(FEEDBACK_COUNTER_TICK_RATE));
    //from RM0433 "Reference Manual" P.1682 Table338
    tim2.set_trigger_source(timer::low_level::TriggerSource::ITR6); // The USB SOF signal.
    tim2.set_slave_mode(timer::low_level::SlaveMode::RESET_MODE);

    tim2.set_input_ti_selection(TIMER_CHANNEL, timer::low_level::InputTISelection::TRC);
    tim2.set_input_capture_prescaler(TIMER_CHANNEL, 0);
    tim2.set_input_capture_filter(TIMER_CHANNEL, timer::low_level::FilterValue::FCK_INT_N8);
    tim2.enable_channel(TIMER_CHANNEL, true);
    tim2.enable_input_interrupt(TIMER_CHANNEL, true);

    tim2.start();

    TIMER.lock(|p| p.borrow_mut().replace(tim2));

    interrupt::TIM2.set_priority(Priority::P0);
    // Unmask the TIM2 interrupt.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    }

    let led = board.user_led;

    // Launch tasks.
    interrupt::UART5.set_priority(Priority::P4);
    let spawner_high = AUDIO_EXECUTOR.start(interrupt::UART5);
    unwrap!(spawner_high.spawn(usb_feedback_task(feedback)));
    unwrap!(spawner_high.spawn(usb_streaming_task(stream, sender)));
    unwrap!(spawner.spawn(audio_receiver_task(interface, receiver, led)));
    unwrap!(spawner.spawn(usb_task(usb_device)));
    unwrap!(spawner.spawn(usb_control_task(control_monitor)));
    unwrap!(spawner.spawn(background_task()));
    unwrap!(spawner.spawn(reporting_task(receiver)));
}

#[embassy_executor::task]
async fn background_task() {
    loop {
        // Spin-wait to keep the CPU from sleeping - which causes current noise
        yield_now().await;
    }
}

#[embassy_executor::task]
async fn reporting_task(
    receiver: channel::Receiver<'static, CriticalSectionRawMutex, i32, AUDIO_BUFF_SIZE>,
) {
    loop {
        Timer::after_millis(100).await;
        info!(
            "buffer: {}, recv: {}, cons: {}, missed: {}, feedback value: {}: {}s/ms, fb count: {}",
            receiver.len(),
            SAMPLES_RECEIVED.load(Ordering::Relaxed),
            SAMPLES_CONSUMED.load(Ordering::Relaxed),
            SAMPLES_MISSED.load(Ordering::Relaxed),
            FEEDBACK_VALUE.load(Ordering::Relaxed),
            FEEDBACK_VALUE.load(Ordering::Relaxed) as f32 / (1 << FEEDBACK_SHIFT) as f32,
            FEEDBACK_COUNT.load(Ordering::Relaxed),
        );
    }
}
