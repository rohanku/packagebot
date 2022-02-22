#!/usr/bin/env python3

# import bluetooth
# import pygame, time, sys
# from pygame.locals import *

import subprocess
import asyncio
import datetime as dt

import serial_asyncio
import aioconsole

DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 2

ANGLE_MIN = 0
ANGLE_MAX = 180
angle = 0
angle_change_rate = 60

sent_angle = 0
prev_t = 0

def run_terminal_command(args, timeout_fail=True):
    p = subprocess.Popen(args)
    try:
        p.wait(CONNECTION_TIMEOUT)
    except subprocess.TimeoutExpired:
        p.kill()
        return -1 if timeout_fail else 0

    return p.returncode

def run_controller():
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Packagebot Controller')
    pygame.mouse.set_visible(0)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while True:
            t = pygame.time.get_ticks();
            delta_t = pygame.time.Clock().tick(30);
            keys = pygame.key.get_pressed()
            if t - prev_t > 20:
                print(angle)
                send_angle = int(angle)
                if send_angle != sent_angle:
                    s.sendall((str(send_angle)+"\n").encode('utf-8'))
                    sent_angle = send_angle
                prev_t = t
            if keys[K_UP]:
                angle = min(ANGLE_MAX, angle + delta_t * angle_change_rate / 1000)
            if keys[K_DOWN]:
                angle = max(ANGLE_MIN, angle - delta_t * angle_change_rate / 1000)
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()

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

