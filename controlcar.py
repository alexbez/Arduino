#
# controlcar.py
#
import bluetooth
import time
from datetime import datetime
import asyncio
from evdev import InputDevice, categorize, ecodes

KEY_UP = 288
KEY_DOWN = 290
KEY_LEFT = 291
KEY_RIGHT = 289
KEY_PANIC = 292
GAMEPAD_DEV = '/dev/input/event0'
CAR_BT_ADDR = "20:14:11:27:32:57"

FILENAME = './robocar.csv'
SERIAL_BT = '/dev/rfcomm0'


async def cmd_helper(dev, btsocket):
    async for ev in dev.async_read_loop():
        cmd='S'
        if ev.type ==ecodes.EV_KEY:
            if ev.value == 0:
                print("Key depressed --> Stop")
                cmd = 'S'
            elif ev.value == 1:
                if ev.code == KEY_UP:
                    cmd = 'F'
                    print("Cmd: Forward")
                if ev.code == KEY_DOWN:
                    cmd = 'B'
                    print("Cmd: Backward")
                if ev.code == KEY_LEFT:
                    cmd = 'L'
                    print("Cmd: Left")
                if ev.code == KEY_RIGHT:
                    cmd = 'R'
                    print("Cmd: Right")
                if ev.code == KEY_PANIC:
                    cmd='S'
                    btsocket.send(cmd)
                    exit(0);
            btsocket.send(cmd)
        await asyncio.sleep(0.1)


async def bt_helper(btsocket, outfile):
    chunks = b""
    while(True):
        chunk = btsocket.recv(1024)
        if chunk == b"":
            raise RuntimeError("Connection closed")
        chunks += chunk
        lines = chunks.splitlines(keepends=True)
        if not lines[-1].endswith(b"\r\n"):
            chunks = lines[-1]
            lines = lines[:-1]
        else:
            chunks = b""
        for line in lines:
            str = line.decode("UTF-8")
            outfile.write( str.rstrip() )
            outfile.write("\r\n")
            print(f"Got: {str}")
            await asyncio.sleep(0.1)
    

async def main(dev, btsocket, outfile):
    tasks.append(asyncio.create_task(cmd_helper(dev, btsocket)))
    tasks.append(asyncio.create_task(bt_helper(btsocket, outfile)))
    print(tasks)
    return await asyncio.gather(*tasks)


cmd = { 'F', 'L', 'R', 'B' }
i = 0

dev = InputDevice(GAMEPAD_DEV)
tasks = []
outfile = open(FILENAME, 'w')
btsocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
btsocket.connect((CAR_BT_ADDR, 1))
print("Car is connected, Gamepad is ready, output file is created")

loop = asyncio.get_event_loop()
results = loop.run_until_complete(main(dev, btsocket, outfile))
print("Completed")

