#!/usr/bin/env bash
# Resolve the Black Magic Probe serial port by USB vendor/product ID.
# BMP exposes two CDC interfaces (1d50:6018); prefer the lowest-numbered ttyACM.
# Prints the port to stdout. Exit 1 if no BMP found.

set -euo pipefail

BMP_VENDOR_ID="1d50"
BMP_PRODUCT_ID="6018"

shopt -s nullglob
candidates=()

for dev in /sys/class/tty/ttyACM*; do
  base=$(basename "$dev")
  vid_file="$dev/device/../idVendor"
  pid_file="$dev/device/../idProduct"
  [[ -f "$vid_file" && -f "$pid_file" ]] || continue
  vid=$(tr -d '[:space:]' <"$vid_file")
  pid=$(tr -d '[:space:]' <"$pid_file")
  if [[ "$vid" == "$BMP_VENDOR_ID" && "$pid" == "$BMP_PRODUCT_ID" ]]; then
    candidates+=("/dev/$base")
  fi
done

if [[ ${#candidates[@]} -gt 0 ]]; then
  IFS=$'\n' sorted=($(printf '%s\n' "${candidates[@]}" | sort -V))
  unset IFS
  printf '%s\n' "${sorted[0]}"
  exit 0
fi

# Fallback: first ttyACM* (better than a hard failure)
for dev in /dev/ttyACM*; do
  if [[ -e "$dev" ]]; then
    printf '%s\n' "$dev"
    exit 0
  fi
done

echo "No ttyACM device found" >&2
exit 1