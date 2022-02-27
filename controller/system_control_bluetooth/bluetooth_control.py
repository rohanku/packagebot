#!/usr/bin/env python3

import bluetooth
import pygame, time, sys
from pygame.locals import *

import subprocess
import asyncio
import serial_asyncio

DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 2

ANGLE_MIN = 0
ANGLE_MAX = 180
angle = 0
angle_increment = 20

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

async def run_controller():
    reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
    writer.write(b"Controller connected.\n")

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Packagebot Controller')
    pygame.mouse.set_visible(0)

    while True:
        data = await reader.readuntil(b'\n')
        print(f'(recv): {data.strip().decode()}')
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    angle = min(ANGLE_MAX, angle + angle_increment)
                if event.key == pygame.K_DOWN:
                    angle = max(ANGLE_MIN, angle - angle-increment)
                print(angle)
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

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
    await asyncio.wait(run_controller())

loop = asyncio.get_event_loop()
task = open_bluetooth_terminal('/dev/cu.ESP32', 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")

