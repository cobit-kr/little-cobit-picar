import time
from adafruit_servokit import ServoKit
import RPi.GPIO as IO


pwmPin = 19
dirPin = 13

pwmPin2 = 12
dirPin2 = 16



IO.setwarnings(False)
IO.setmode(IO.BCM)
IO.setup(pwmPin, IO.OUT)
IO.setup(dirPin,IO.OUT)
IO.setup(pwmPin2, IO.OUT)
IO.setup(dirPin2,IO.OUT)



p = IO.PWM(pwmPin, 100)
p.start(0)

p2 = IO.PWM(pwmPin2, 100)
p2.start(0)

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

try:
    while True:
        p.ChangeDutyCycle(0)
        IO.output(dirPin, True) 
        p2.ChangeDutyCycle(0)
        IO.output(dirPin2, True) 
        kit.servo[0].angle = 0
        print("test1")
        time.sleep(1)
        kit.servo[0].angle = 90
        time.sleep(1)
        print("test2")
        kit.servo[0].angle = 180
        time.sleep(1)
        print("test3")
        kit.servo[0].angle = 90
        time.sleep(1)
        print("test4")

        #p.ChangeDutyCycle(100)
        #IO.output(dirPin, False) 
        #p2.ChangeDutyCycle(100)
        #IO.output(dirPin2, False) 
        time.sleep(2)
except KeyboardInterrupt:
    IO.output(dirPin, False) 
    IO.output(pwmPin, False)
    IO.output(dirPin2, False) 
    IO.output(pwmPin2, False)
    IO.cleanup() 
