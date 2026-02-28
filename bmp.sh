#!/usr/bin/env bash
# set -x

script_dir=$(dirname $(realpath $0))
repo_pth=$(git rev-parse --show-toplevel)
example_name="usb_uac"
elf_pth="${repo_pth}/${2:-target/thumbv7em-none-eabihf/debug/examples/$example_name}"
bmpgdb_pth=/dev/ttyBmpGdb
bmprtt_pth=/dev/ttyBmpTarg

bmputil-cli probe info &> /dev/null

map_rtt_entry=$(arm-none-eabi-nm $elf_pth | grep -E "^[0-9a-f]{8} D _SEGGER_RTT$")
if [[ ${#map_rtt_entry} -gt 0 ]]; then
  rtt_ram=$(awk '{print "0x"$1}' <<< $map_rtt_entry)
  rtt_ram+=$(printf " 0x%x" $(($rtt_ram + 0x2000)))
fi

while getopts gr opt; do
 case ${opt} in
  g)
    # -ex "tui en" \
    # -ex "fs cmd" \
    arm-none-eabi-gdb \
    -ex "tar ext ${bmpgdb_pth}" \
    -ex "set pag off" \
    -ex "set confirm off" \
    -ex "mon swd_scan" \
    -ex "att 1" \
    -ex "mon rtt" \
    -ex "mon rtt ram ${rtt_ram}" \
    -ex load \
    -ex start \
    ${elf_pth} 
    exit 0
    ;;
  r)
    socat -u ${bmprtt_pth},raw,echo=0 STDOUT | defmt-print -e ${elf_pth} --log-format "{t} [{L}] {m}:{l} {s}" -w
    exit 0
    ;;
  *) exit 0;;
  esac
done
