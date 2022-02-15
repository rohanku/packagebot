#!/usr/bin/env python3

import socket
import pygame, time, sys
from pygame.locals import *

HOST = '192.168.1.14'  # The server's hostname or IP address
PORT = 80        # The port used by the server

pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Pygame Keyboard Test')
pygame.mouse.set_visible(0)



with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.connect((HOST, PORT))
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				sys.exit()
			if (event.type == KEYUP) or (event.type == KEYDOWN):
				if event.mod == pygame.KMOD_NONE:
					print('No modifier keys were in a pressed state when this '
							'event occurred.')
				else:
					if event.mod & pygame.KMOD_LSHIFT:
						print('Left shift was in a pressed state when this event '
								'occurred.')
					if event.mod & pygame.KMOD_RSHIFT:
						print('Right shift was in a pressed state when this event '
								'occurred.')
					if event.mod & pygame.KMOD_SHIFT:
						print('Left shift or right shift or both were in a '
								'pressed state when this event occurred.')

