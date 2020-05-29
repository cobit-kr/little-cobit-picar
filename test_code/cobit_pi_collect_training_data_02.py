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
from camera import Camera

PID_MICROBIT = 516
VID_MICROBIT = 3368
TIMEOUT= 0.1

IR_LEFT = 76            # 'L'
IR_RIGHT = 82           # 'R'
RECORD_DATA = 80        # 'P'

img_size = 320*120

img_roi = None
img = None

# serial thread 
class SerialThread(Thread):
  global img_roi
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
    self.is_serial_running = False
    self.daemon = True

    # Create label
    self.k = np.zeros((3, 3), 'float')
    for i in range(2):
        self.k[i, i] = 1
    print(self.k)

    # data set array 
    self.img_ml = np.empty((0, img_size))
    self.dir_ml = np.empty((0, 3))

    self.roi_cnt = 0

  def run(self):

    while True:
      time.sleep(0.1)
      if self.seq.isOpen() == True:  
        try:
          byte_in = self.seq.inWaiting()
          if byte_in > 0:  
            try:
              cmd = self.seq.read(byte_in)
              cmd_rev = cmd[0]
              if img_roi.any():
                w, h = img_roi.shape
                if cmd_rev == IR_LEFT or cmd_rev == IR_RIGHT:
                  temp_array = img_roi.reshape(1, w*h).astype(np.float32)
                  if cmd_rev == IR_RIGHT:
                    print("RIGHT")
                    self.img_ml = np.vstack((self.img_ml, temp_array))
                    self.dir_ml = np.vstack((self.dir_ml, self.k[0]))
                  elif cmd_rev == IR_LEFT:
                    print("LEFT")
                    self.img_ml = np.vstack((self.img_ml, temp_array))
                    self.dir_ml = np.vstack((self.dir_ml, self.k[1]))
                  self.roi_cnt = self.roi_cnt+1
                  #cv2.imwrite('roi_'+str(self.roi_cnt)+'.jpg', img_roi)  
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
                self.dir_ml = np.empty((0, 2))

            
            except AttributeError:
              print("attr error")
        except IOError:
            print("IO error")
  
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

# camera thread 
class CameraThread(Thread):
  global img_roi

  def __init__(self):
    Thread.__init__(self)
    self.daemon = True

  def run(self):
    # Activate camera 
    cap = cv2.VideoCapture(0)

    # Set camera image size
    cap.set(3,320)
    cap.set(4,240)
    while True:
      ret, img = cap.read()
      if(ret):
        #cv2.imshow('color', img)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('gray', img_gray)

        # reshaping camera image 
        height, width = img_gray.shape
        img_roi = img_gray[int(height/2):height, : ]
        cv2.imshow('roi', img_roi)

      if cv2.waitKey(1) & 0xff == ord('q'):
        break

    cap.release()
    cv2.destroyAllWindows()

# Flask 

# App main 

# Start serial thread 
serial_thread = SerialThread()
serial_thread.start()

camera_thread = CameraThread()
camera_thread.start()

print('looking for microbit')
if serial_thread.find_comport(PID_MICROBIT, VID_MICROBIT, 115200) == False:
    print('microbit not found')
    sys.exit()
print('opening and monitoring microbit port')
serial_thread.open_port()

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
