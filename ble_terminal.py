import asyncio
import sys
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "Mini-FT8"

UART_RX_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
UART_TX_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"


def handle_notify(_, data: bytearray):
    try:
        print(data.decode(errors="ignore"), end="", flush=True)
    except Exception:
        print(data)


async def main():
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        if d.name == DEVICE_NAME:
            target = d
            break

    if not target:
        print("❌ Device not found")
        for d in devices:
            print(d.name, d.address)
        return

    async with BleakClient(target.address) as client:
        print("Connected")
        await client.start_notify(UART_TX_UUID, handle_notify)
        print("Notify enabled — type below:\n")

        loop = asyncio.get_running_loop()

        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            await client.write_gatt_char(
                UART_RX_UUID,
                line.encode(),
                response=True
            )

asyncio.run(main())
