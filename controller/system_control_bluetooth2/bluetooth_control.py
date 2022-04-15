#!/usr/bin/env python3

import pygame, time, sys
from pygame.locals import *

import subprocess
import asyncio
import serial_asyncio
import os

DEVICE_NAME = 'ESP32'
DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 10

ANGLE_MIN = 0
ANGLE_MAX = 180
ANGLE_INCREMENT = 20

angle1 = 0
angle2 = 0

def run_terminal_command(args):
    p = subprocess.Popen(args)
    try:
        p.wait(CONNECTION_TIMEOUT)
    except subprocess.TimeoutExpired:
        p.kill()

    return p.returncode

async def receive(reader):
    print("receiving...")
    while True:
        data = await reader.readuntil(b'\n')
        print(f'(recv): {data.strip().decode()}')

async def run_controller(writer):
    global angle1, angle2
    writer.write(b"Controller connected.\n")

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Packagebot Controller')
    pygame.mouse.set_visible(0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    angle1 = min(ANGLE_MAX, angle1 + ANGLE_INCREMENT)
                    writer.write(b'{"command":0,"args":[0,%d]}\n' % angle1)
                    print("Increasing angle1: ", angle1)
                if event.key == pygame.K_DOWN:
                    angle1 = max(ANGLE_MIN, angle1 - ANGLE_INCREMENT)
                    writer.write(b'{"command":0,"args":[0,%d]}\n' % angle1)
                    print("Decreasing angle1: ", angle1)
                if event.key == pygame.K_LEFT:
                    angle2 = min(ANGLE_MAX, angle2 + ANGLE_INCREMENT)
                    writer.write(b'{"command":0,"args":[1,%d]}\n' % angle2)
                    print("Increasing angle2: ", angle2)
                if event.key == pygame.K_RIGHT:
                    angle2 = max(ANGLE_MIN, angle2 - ANGLE_INCREMENT)
                    writer.write(b'{"command":0,"args":[1,%d]}\n' % angle2)
                    print("Decreasing angle2: ", angle2)
                if event.key == pygame.K_a:
                    writer.write(b'{"command":1,"args":[0,255]}\n')
                    print("Motor forward")
                if event.key == pygame.K_s:
                    writer.write(b'{"command":1,"args":[2,0]}\n')
                    print("Motor stop")
                if event.key == pygame.K_d:
                    writer.write(b'{"command":1,"args":[1,255]}\n')
                    print("Motor backward")
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
        await asyncio.sleep(0.001)

async def open_bluetooth_terminal(port, baudrate):
    print("Checking if device at %s is paired..." % DEVICE_MAC_ADDRESS)
    if not os.path.exists(port):
        print("Pairing with device at %s..." % DEVICE_MAC_ADDRESS)
        rc = run_terminal_command(["blueutil", "--pair", DEVICE_MAC_ADDRESS])
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
    controller = run_controller(writer)
    await asyncio.wait([receiver, controller])

loop = asyncio.get_event_loop()
task = open_bluetooth_terminal('/dev/cu.%s' % DEVICE_NAME, 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")

