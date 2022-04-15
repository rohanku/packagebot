#!/usr/bin/env python3

import subprocess
import asyncio
import serial_asyncio
import csv
import matplotlib.pyplot as plt
import os

device = "/dev/cu.ESP32"
DEVICE_MAC_ADDRESS = '40-f5-20-45-22-d2'
CONNECTION_TIMEOUT = 10

measurements = ['roll', 'pitch', 'yaw', 'xaccel', 'yaccel', 'zaccel']

data_dict = { measurement: [] for measurement in measurements }
rows = []

def run_terminal_command(args):
    p = subprocess.Popen(args)
    try:
        p.wait(CONNECTION_TIMEOUT)
    except subprocess.TimeoutExpired:
        p.kill()

    return p.returncode

async def read_data(port, baudrate):
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
    with open('data.txt', 'wb') as data_file:
        print("Opening connection")
        reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
        print("Connection opened")
        writer.write(b'\n')
        data = await reader.readuntil(b'\n')
        data_file.write(data)
        counter = int(data)
        for measurement in measurements:
            data = await reader.readuntil(b'\n')
            data_file.write(data)
            for _ in range(counter):
                data = await reader.readuntil(b'\n')
                data_file.write(data)
                data_dict[measurement].append(float(data))

    plot1_measurements = measurements[:3]
    for measurement in plot1_measurements:
        plt.plot(data_dict[measurement])
    plt.legend(plot1_measurements)

    plt.figure()
    plot2_measurements = measurements[3:]
    for measurement in plot2_measurements:
        plt.plot(data_dict[measurement])
    plt.legend(plot2_measurements)
    plt.show()

loop = asyncio.get_event_loop()
task = read_data(device, 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")

