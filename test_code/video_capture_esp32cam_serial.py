import cv2 as cv
import numpy as np
#from urllib.request import urlopen
import urllib.request
#import socket
import serial
import serial.tools.list_ports as list_ports
import sys
from threading import Thread, Event
import queue as Queue
import time

PID_MICROBIT = 516
VID_MICROBIT = 3368
TIMEOUT= 0.1

IR_LEFT = 76
IR_RIGHT = 82

img_cvt_cnt = 0


stream= urllib.request.urlopen("http://192.168.0.13:81/stream")
byte_array=b''

img_cvt = None

class SerialThread(Thread):
    global img_cvt_cnt
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

    def run(self):
        #pass
        #'''
        img_cvt_cnt  = 0
        turn = []
        while True:
            time.sleep(0.05)
            if self.seq.isOpen() == True:  
                try:
                    if self.seq.inWaiting():
                        try:
                            cmd = self.seq.readline()
                            cmd_rev = cmd[0]
                            if img_cvt.any():
                                if cmd_rev == IR_RIGHT or cmd_rev == IR_LEFT:
                                    img_cvt_cnt = img_cvt_cnt+1
                                    if cmd_rev == IR_RIGHT:
                                        turn.append('RIGHT')
                                    elif cmd_rev == IR_LEFT:
                                        turn.append('LEFT')
                                    print(str(cmd_rev)+" "+str(img_cvt_cnt)+" "+str(len(turn)))
                                    print(turn)
                                    cv.imwrite('img_cvt_'+str(img_cvt_cnt)+'.jpg', img_cvt)
                        except AttributeError:
                            print("attr error")
                except IOError:
                    print("IO error")
        #'''            
        
    def find_comport(self, pid, vid, baud):
        #ser_port = serial.Serial(timeout=TIMEOUT)
        #ser_port.baudrate = baud
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
            img = cv.imdecode(jpeg_img,cv.IMREAD_UNCHANGED)
            img_cvt = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            #detected_object = haar_class.detectMultiScale(img_cvt, 1.3, 5)
    
            #for i in detected_object:
            #    message = b'stop\n'
                #sock.sendall(message)
            #    cv.rectangle(img, (i[0], i[1]), (i[0]+i[2], i[1]+i[3]), (255, 0, 0), 2)
       
            cv.imshow("a",img)
    k=cv.waitKey(1)
    if k & 0xFF==ord('q'):
        break
    
   
cv.destroyAllWindows()