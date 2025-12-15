import asyncio
import sys
from bleak import BleakScanner, BleakClient

# ========= CONFIG =========

DEVICE_NAME = "Mini-FT8"

UART_RX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"  # write
UART_TX_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"  # notify

# ========= STATE =========

capture_active = False
capture_file = None
ignore_first_line = False

exit_requested = False
exit_confirmed = False
exit_event = asyncio.Event()

# ========= NOTIFY HANDLER =========

def handle_notify(_, data: bytearray):
    global capture_active, capture_file, ignore_first_line
    global exit_requested, exit_confirmed

    if exit_confirmed:
        return  # hard stop

    text = data.decode(errors="ignore")

    for line in text.splitlines(keepends=True):
        s = line.strip()

        # ----- EXIT MODE -----
        if exit_requested:
            if s.upper().startswith("OK"):
                print("\n[Remote exit OK — closing]")
                exit_confirmed = True
                exit_event.set()
            # ignore EVERYTHING else during exit
            continue

        # ----- READ CAPTURE MODE -----
        if capture_active:
            if ignore_first_line:
                ignore_first_line = False
                continue

            if s.upper().startswith("OK"):
                capture_active = False
                capture_file.close()
                capture_file = None
                print("\n[Capture finished]")
                continue

            capture_file.write(line)
            continue

        # ----- NORMAL TERMINAL OUTPUT -----
        print(line, end="", flush=True)

# ========= MAIN =========

async def main():
    global capture_active, capture_file, ignore_first_line
    global exit_requested

    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = next((d for d in devices if d.name == DEVICE_NAME), None)
    if not target:
        print("❌ Device not found")
        print("Found devices:")
        for d in devices:
            print(f"  {d.name} [{d.address}]")
        return

    async with BleakClient(target.address) as client:
        print("Connected")
        await client.start_notify(UART_TX_UUID, handle_notify)
        print("Ready. Type commands:\n")

        loop = asyncio.get_running_loop()

        while not exit_event.is_set():
            cmd = await loop.run_in_executor(None, sys.stdin.readline)
            if not cmd:
                break

            cmd = cmd.strip()
            if not cmd:
                continue

            # ----- READ COMMAND -----
            if cmd.upper().startswith("READ "):
                filename = cmd.split(" ", 1)[1]
                local_name = filename.split("/")[-1]

                capture_file = open(local_name, "w", newline="")
                capture_active = True
                ignore_first_line = True
                print(f"[Capturing → {local_name}]")

            # ----- EXIT COMMAND -----
            if cmd.upper() == "EXIT":
                exit_requested = True
                print("[Waiting for remote OK then exit]")

            await client.write_gatt_char(
                UART_RX_UUID,
                (cmd + "\n").encode(),
                response=True
            )

        # Clean shutdown
        await client.stop_notify(UART_TX_UUID)

    print("Disconnected. Bye.")

# ========= ENTRY =========

asyncio.run(main())
