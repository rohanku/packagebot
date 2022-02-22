#!/usr/bin/env python3

import socket
import pygame, time, sys
from pygame.locals import *

HOST = '192.168.1.14'  # The server's hostname or IP address
PORT = 80        # The port used by the server

ANGLE_MIN = 0
ANGLE_MAX = 180
angle = 0
angle_change_rate = 60

sent_angle = 0
prev_t = 0

pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Pygame Keyboard Test')
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
