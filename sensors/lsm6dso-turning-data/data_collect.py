#!/usr/bin/env python3

import asyncio
import serial_asyncio
import csv
import matplotlib.pyplot as plt

device = "/dev/tty.usbserial-01F4AC71"

measurements = ['roll', 'pitch', 'yaw', 'xaccel', 'yaccel', 'zaccel']

data_dict = { measurement: [] for measurement in measurements }
rows = []

async def read_data(port, baudrate):
    with open('data.txt', 'wb') as data_file:
        reader, writer = await serial_asyncio.open_serial_connection(url=port, baudrate=baudrate)
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

    for measurement in measurements:
        plt.plot(data_dict[measurement])
    plt.legend(measurements)
    plt.show()


loop = asyncio.get_event_loop()
task = read_data(device, 115200)
try:
    loop.run_until_complete(task)
    loop.close()
except KeyboardInterrupt:
    print("Exiting...")

