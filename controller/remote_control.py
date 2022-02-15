#!/usr/bin/env python3

import socket
import time

HOST = '192.168.1.14'  # The server's hostname or IP address
PORT = 80        # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b'1')
    time.sleep(3)
    s.sendall(b'0')

