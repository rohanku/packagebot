#!/usr/bin/env python3

import subprocess
import asyncio
import serial_asyncio
import aioconsole

DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 2

def run_terminal_command(args, timeout_fail=True):
    p = subprocess.Popen(args)
    try:
        p.wait(CONNECTION_TIMEOUT)
    except subprocess.TimeoutExpired:
        p.kill()
        return -1 if timeout_fail else 0

    return p.returncode

async def receive(reader):
    while True:
        data = await reader.readuntil(b'\n')
        print(f'(recv): {data.strip().decode()}')

async def send(writer):
    writer.write(b"Connection initialized.\n")
    stdin, _ = await aioconsole.get_standard_streams()
    async for line in stdin:
        data = line.strip()
        if not data:
            continue
        writer.write(line)


async def open_bluetooth_terminal(port, baudrate):
    print("Setting up device at %s..." % DEVICE_MAC_ADDRESS)
    rc = run_terminal_command(["blueutil", "--pair", DEVICE_MAC_ADDRESS], timeout_fail=False)
    if rc != 0:
        print("Failed to pair with device. Exiting...")
        return
    print("Connecting to device at %s..." % DEVICE_MAC_ADDRESS)
    rc = run_terminal_command(["blueutil", "--connect", DEVICE_MAC_ADDRESS])
    if rc != 0:
        print("Failed to connect to device. Exiting...")
        return
    print("Connected successfully.")
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    receiver = receive(reader)
    sender = send(writer)
    await asyncio.wait([receiver, sender])

loop = asyncio.get_event_loop()
task = open_bluetooth_terminal('/dev/cu.ESP32', 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")
