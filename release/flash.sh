#!/usr/bin/env bash
# Bash helper to flash Mini-FT8 on Ubuntu (8MB flash, 4MB SPIFFS)
# Usage:
#   1) Install esptool (recommended):
#        sudo apt install -y pipx
#        pipx ensurepath && exec $SHELL
#        pipx install esptool
#   2) Edit PORT below (e.g., /dev/ttyACM0 or /dev/ttyUSB0)
#   3) chmod +x ./flash.sh
#   4) ./flash.sh

set -euo pipefail

PORT="/dev/ttyACM0"      # TODO: change to your port: /dev/ttyACM0 or /dev/ttyUSB0
BAUD=460800
CHIP="esp32s3"

BOOTLOADER="bootloader.bin"
PARTITION="partition-table.bin"
APP="mini_ft8.bin"

# Optional: if you also have these, you can add them to the flash args below
# OTA_DATA="ota_data_initial.bin"     # often @ 0xe000
# SPIFFS="spiffs.bin"                 # offset varies; only flash if you KNOW the offset

for f in "$BOOTLOADER" "$PARTITION" "$APP"; do
  [[ -f "$f" ]] || { echo "ERROR: Missing $f (are you in the release folder?)"; exit 1; }
done

# Helpful hint: ensure you have permissions for the serial port
# sudo usermod -aG dialout $USER  # then log out/in

echo "Flashing $CHIP on $PORT @ $BAUD..."
esptool --chip "$CHIP" -p "$PORT" -b "$BAUD" \
  --before default_reset --after hard_reset \
  write_flash \
  --flash-mode dio --flash-size 8MB --flash-freq 80m \
  0x0    "$BOOTLOADER" \
  0x8000 "$PARTITION" \
  0x10000 "$APP"

echo "Done."

