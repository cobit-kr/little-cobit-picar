import cv2 as cv
import numpy as np
from urllib.request import urlopen
import os
import datetime
import time
import sys
import socket
import requests


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

#face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
face_cascade = cv.CascadeClassifier('stop_sign.xml')
#eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

while True: 
    stop_flag = False
    bts+=stream.read(CAMERA_BUFFRER_SIZE)
    jpghead=bts.find(b'\xff\xd8')
    jpgend=bts.find(b'\xff\xd9')
    if jpghead>-1 and jpgend>-1:
        jpg=bts[jpghead:jpgend+2]
        bts=bts[jpgend+2:]
        buff = np.frombuffer(jpg,dtype=np.uint8)
        if buff.any():
            img=cv.imdecode(buff,cv.IMREAD_UNCHANGED)
            v=cv.flip(img,0)
            h=cv.flip(img,1)
            p=cv.flip(img,-1)        
            frame=p
            h,w=frame.shape[:2]
            #img=cv.resize(frame,(800,600))
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)
            for i in faces:
                stop_flag = True
                count = 50
                cv.rectangle(img, (i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (255, 0, 0), 2)
            cv.imshow("a",img)
    k=cv.waitKey(1)
    if k & 0xFF==ord('q'):
        break

    if count > 0:
        count = count - 1

    if stop_flag:
        print("stop")
        message = b'stop\n'
        #sock.sendall(message)
    else:
        if count == 0:
            print("")
            #message = b'run\n' 
    
cv.destroyAllWindows()