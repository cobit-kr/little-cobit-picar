import cv2 as cv
import numpy as np
import urllib.request
#import socket
import serial
import serial.tools.list_ports as list_ports
import sys
from threading import Thread, Event
import queue as Queue
import time
import os

PID_MICROBIT = 516
VID_MICROBIT = 3368
TIMEOUT= 0.1

IR_LEFT = 76            # 'L'
IR_RIGHT = 82           # 'R'
RECORD_DATA = 80        # 'P'
IR_FORWARD = 70         # 'F'
#ROBOT_START = 83        # 'S'

roi = None


stream=urllib.request.urlopen("http://192.168.0.13:81/stream")
byte_array=b''

img_cvt = None

# Image size 
img_size = 148*400

class SerialThread(Thread):
    global roi
    def __init__(self):
        Thread.__init__(self)

        self.seq = serial.Serial(
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        self.seq.port = "None"
       
        self.queue = serial_queue
        self.is_serial_running = False
        self.daemon = True

        # Create label
        self.k = np.zeros((3, 3), 'float')
        for i in range(3):
            self.k[i, i] = 1
        print(self.k)

        # data set array 
        self.img_ml = np.empty((0, img_size))
        self.dir_ml = np.empty((0, 3))

        self.roi_cnt = 0

    def run(self):
        #pass
        #'''
        turn = []
        while True:
            time.sleep(0.05)
            if self.seq.isOpen() == True:  
                try:
                    if self.seq.inWaiting():
                        try:
                            cmd = self.seq.readline()
                            cmd_rev = cmd[0]
                            if roi.any():
                                if cmd_rev == IR_LEFT or cmd_rev == IR_RIGHT or cmd_rev == IR_FORWARD:
                                    temp_array = roi.reshape(1, int(height/2)*width).astype(np.float32)
                                    if cmd_rev == IR_RIGHT:
                                        print("RIGHT")
                                        turn.append("RIGHT")
                                        self.img_ml = np.vstack((self.img_ml, temp_array))
                                        self.dir_ml = np.vstack((self.dir_ml, self.k[0]))
                                    elif cmd_rev == IR_LEFT:
                                        print("LEFT")
                                        turn.append("LEFT")
                                        self.img_ml = np.vstack((self.img_ml, temp_array))
                                        self.dir_ml = np.vstack((self.dir_ml, self.k[1]))
                                    elif cmd_rev == IR_FORWARD:
                                        print("FORWARD")
                                        turn.append("FORWARD")
                                        self.img_ml = np.vstack((self.img_ml, temp_array))
                                        self.dir_ml = np.vstack((self.dir_ml, self.k[2]))
                                    self.roi_cnt = self.roi_cnt+1
                                    #cv.imwrite('roi_'+str(self.roi_cnt)+'.jpg', roi)  
                                elif cmd_rev == RECORD_DATA:
                                    print("record data")
                                    file_name = str(int(time.time()))
                                    directory = "training_data"
                                    if not os.path.exists(directory):
                                        os.makedirs(directory)
                                    try:
                                        np.savez(directory + '/' + file_name + '.npz', train=self.img_ml, train_labels=self.dir_ml)
                                    except IOError as e:
                                        print(e)
                                    self.img_ml = np.empty((0, img_size))
                                    self.dir_ml = np.empty((0, 3))


                        except AttributeError:
                            print("attr error")
                except IOError:
                    print("IO error")
        #'''  
        
    def find_comport(self, pid, vid, baud):
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
                self.seq.port = str(p.device)
                return True
        return False

    def open_port(self):
        if self.seq.isOpen() == False:
            self.seq.open()

serial_queue = Queue.Queue()
serial_thread = SerialThread()
serial_thread.start()

print('looking for microbit')
if serial_thread.find_comport(PID_MICROBIT, VID_MICROBIT, 115200) == False:
    print('microbit not found')
    sys.exit()
print('opening and monitoring microbit port')
serial_thread.open_port()



while True: 
    
    start = cv.getTickCount()
    byte_array+=stream.read(4096) #2048 best working 
    jpghead=byte_array.find(b'\xff\xd8')
    jpgend=byte_array.find(b'\xff\xd9')
    if jpghead>-1 and jpgend>-1:
        is_jpeg = True
    else:
        is_jpeg = False
        #print("JPEG fail")
    if is_jpeg:  
        jpg=byte_array[jpghead:jpgend+2]
        byte_array=byte_array[jpgend+2:]
        jpeg_img = np.frombuffer(jpg,dtype=np.uint8)
        is_OK = jpeg_img.any()
        if is_OK:
            img = cv.imdecode(jpeg_img, cv.IMREAD_GRAYSCALE)
            height, width = img.shape
            #print(str(height)+" "+str(width))
            roi = img[int(height/2):height, : ]
            cv.imshow("b",roi)
           
    k=cv.waitKey(1)
    if k & 0xFF==ord('q'):
        break
    end = cv.getTickCount()
    #print("Streaming duration: , %.2fs" % ((end - start) / cv.getTickFrequency()))
    
   
cv.destroyAllWindows()
