#!/usr/bin/env python
from flask import Flask, render_template, Response,  request, redirect, url_for
from imutils.video import VideoStream
import threading
import serial
import serial.tools.list_ports as list_ports
import time
import imutils
import cv2
import numpy as np
import sys
import os 
import argparse

outputFrame = None
lock = threading.Lock()

seq = None

IR_LEFT = 76            # 'L'
IR_RIGHT = 82           # 'R'
RECORD_DATA = 80        # 'P'
IR_FORWARD = 70         # 'F'

PID_MICROBIT = 516
VID_MICROBIT = 3368
TIMEOUT= 0.1

img_size = 320*120

def find_comport(serial, pid, vid, baud):
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
            serial.port = str(p.device)
            return True
    return False

def collect_data():
    global vs, outputFrame, lock

    while True:
        frame = vs.read()
        frame2 = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_CUBIC)
        gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        gray_half = gray[120:240, : ]
        gray_flip = cv2.flip(gray_half, 0)

        with lock:
            outputFrame = gray_flip.copy()

def serial_process():
    global lock, outputFrame
    seq = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )
    #seq.port = "/dev/ttyACM0"
    print('looking for microbit')
    if find_comport(seq, PID_MICROBIT, VID_MICROBIT, 115200) == False:
        print('microbit not found')
        sys.exit()
    print('opening and monitoring microbit port')
    seq.open()

    k = np.zeros((3, 3), 'float')
    for i in range(3):
        k[i, i] = 1
    print(k)

    img_ml = np.empty((0, img_size))
    dir_ml = np.empty((0, 3))

    roi_cnt = 0
    
    while True:
        #time.sleep(0.05)
        if seq.isOpen() == True:  
            try:
                if seq.inWaiting():
                    try:
                        cmd = seq.readline()
                        cmd_rev = cmd[0]
                        #with lock:
                        if outputFrame.any():
                            if cmd_rev == IR_LEFT or cmd_rev == IR_RIGHT or cmd_rev == IR_FORWARD:
                                height, width = outputFrame.shape
                                temp_array = outputFrame.reshape(1, height*width).astype(np.float32)
                                if cmd_rev == IR_RIGHT:
                                    print("RIGHT")
                                    img_ml = np.vstack((img_ml, temp_array))
                                    dir_ml = np.vstack((dir_ml, k[0]))
                                elif cmd_rev == IR_LEFT:
                                    print("LEFT")
                                    img_ml = np.vstack((img_ml, temp_array))
                                    dir_ml = np.vstack((dir_ml, k[1]))
                                elif cmd_rev == IR_FORWARD:
                                    print("FORWARD")
                                    img_ml = np.vstack((img_ml, temp_array))
                                    dir_ml = np.vstack((dir_ml, k[2]))
                                roi_cnt = roi_cnt+1
                                #cv.imwrite('roi_'+str(self.roi_cnt)+'.jpg', roi)  
                            elif cmd_rev == RECORD_DATA:
                                print("record data")
                                file_name = str(int(time.time()))
                                directory = "training_data"
                                if not os.path.exists(directory):
                                    os.makedirs(directory)
                                try:
                                    np.savez(directory + '/' + file_name + '.npz', train=img_ml, train_labels=dir_ml)
                                except IOError as e:
                                    print(e)
                                img_ml = np.empty((0, img_size))
                                dir_ml = np.empty((0, 3))
                           
                    except AttributeError:
                        print("attr error")
            except IOError:
                print("IO error")



app = Flask(__name__)

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--picamera", type=int, default=-1,
	help="whether or not the Raspberry Pi camera should be used")
args = vars(ap.parse_args())

vs = VideoStream(usePiCamera=args["picamera"] > 0).start()
#vs = VideoStream(src=0).start()

time.sleep(2.0)

@app.route('/')
def index():
    return render_template("index.html")

def generate():
    global outputFrame, lock
    while True:
        with lock:
            if outputFrame is None:
                continue

            flag, encodedImage = cv2.imencode(".jpg", outputFrame)

            if not flag:
                continue

        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encodedImage) + b'\r\n')
        

@app.route('/video_feed')
def video_feed():
    return Response(generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/terminate', methods=['POST'])
def terminate_script():
    print("Terminate script")
    exit()
    return render_template('index.html')

if __name__ == '__main__':
    t = threading.Thread(target=collect_data)
    t.daemon = True
    t.start()
    s = threading.Thread(target=serial_process)
    s.daemon = True
    s.start()
   
    app.run(host='192.168.0.18', port='8000', debug=True, threaded=True, use_reloader=False) 

vs.stop()