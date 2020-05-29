import cv2 as cv
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys
import socket
import requests
import pygame
from pygame.locals import *


count = 0

#change to your ESP32-CAM ip
url="http://192.168.0.13:81/stream"
CAMERA_BUFFRER_SIZE=4096

stream=urlopen(url)
bts=b''
              
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 

host = "192.168.0.13" #ESP32 IP in local network
port = 82             #ESP32 Server Port    
 
sock.connect((host, port))

pygame.init()
pygame.display.set_mode((250, 250))

send_inst = True

while send_inst: 
    for event in pygame.event.get():
        if event.type == KEYDOWN:
            key_input = pygame.key.get_pressed()

            # simple orders
            if key_input[pygame.K_UP]:
                print("Forward")
                message = b'a'
                sock.sendall(message)
            elif key_input[pygame.K_DOWN]:
                print("Reverse")
                message = b'b'
                sock.sendall(message)
            elif key_input[pygame.K_RIGHT]:
                print("Right")
                message = b'c'
                sock.sendall(message)
            elif key_input[pygame.K_LEFT]:
                print("Left")
                message = b'd'
                sock.sendall(message)
            elif key_input[pygame.K_p]:
                print("Stop")
                message = b'p'
                sock.sendall(message)
            # exit
            elif key_input[pygame.K_x] or key_input[pygame.K_q]:
                print("Exit")
                send_inst = False
                break

        elif event.type == pygame.KEYUP:
            pass