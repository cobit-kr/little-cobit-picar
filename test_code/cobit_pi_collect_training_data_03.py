import cv2 
import numpy as np 
from threading import Thread, Event
import serial
import serial.tools.list_ports as list_ports
import time 
import sys
import os 

# experimental 
from flask import Flask, render_template, Response

PID_MICROBIT = 516
VID_MICROBIT = 3368
TIMEOUT= 0.1

IR_LEFT = 76            # 'L'
IR_RIGHT = 82           # 'R'
RECORD_DATA = 80        # 'P'

img_size = 320*240

img_roi = None

seq = None

def find_comport( pid, vid, baud):
  ports = list(list_ports.comports())
  print('scanning ports')
  for p in ports:
    print('port: {}'.format(p))
    try:
      print('pid: {} vid: {}'.format(p.pid, p.vid))
    except AttributeError:
      continue
    if (p.pid == pid) and (p.vid == vid):
      print('found target device pid: {} vid: {} port: {}'.format(
        p.pid, p.vid, p.device))
      seq.port = str(p.device)
      return True
  return False

seq = serial.Serial(
      baudrate=115200,
      parity=serial.PARITY_NONE,
      stopbits=serial.STOPBITS_ONE,
      bytesize=serial.EIGHTBITS,
      timeout=1
    )

k = np.zeros((3, 3), 'float')
for i in range(3):
  k[i, i] = 1
print(k)

print('looking for microbit')
if find_comport(PID_MICROBIT, VID_MICROBIT, 115200) == False:
    print('microbit not found')
    sys.exit()
print('opening and monitoring microbit port')
seq.open()

img_ml = np.empty((0, img_size))
dir_ml = np.empty((0, 3))

# Activate camera 
cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)

while True:
  ret, img = cap.read()
  if(ret):
    cv2.imshow('color', img)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', img_gray)

    # reshaping camera image 
    height, width = img_gray.shape
    img_roi = img_gray[int(height/2):height, : ]
    cv2.imshow('roi', img_roi)

    # read serial
    '''
    bytesToRead = seq.inWaiting()
    if bytesToRead > 0:
      cmd = seq.read(bytesToRead)
      print(cmd)
    '''
    
    cmd = seq.read(1)
    if cmd == b'L':
      print('LEFT')
    elif cmd == b'R':
      print('RIGHT')
    
   
    if cv2.waitKey(1) & 0xff == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()