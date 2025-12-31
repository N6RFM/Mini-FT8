# PowerShell helper to flash Mini-FT8 (8MB flash, 4MB SPIFFS)
# Usage:
#   1) pip install esptool
#   2) Edit $Port below to match your device (e.g., COM11)
#   3) .\flash.ps1

$ErrorActionPreference = "Stop"

$Port = "COM11"  # TODO: change to your COM port
$Baud = 460800
$Chip = "esp32s3"

$Bootloader = "bootloader.bin"
$Partition  = "partition-table.bin"
$App        = "mini_ft8.bin"

if (!(Test-Path $Bootloader) -or !(Test-Path $Partition) -or !(Test-Path $App)) {
    Write-Error "Build artifacts not found. Run 'idf.py build' first."
}

$Args = @(
    "--chip", $Chip,
    "-p", $Port,
    "-b", $Baud,
    "--before", "default_reset",
    "--after", "hard_reset",
    "write_flash",
    "--flash_mode", "dio",
    "--flash_size", "8MB",
    "--flash_freq", "80m",
    "0x0",     $Bootloader,
    "0x8000",  $Partition,
    "0x10000", $App
)

Write-Host "Flashing $Chip on $Port @ $Baud..."
esptool @Args
