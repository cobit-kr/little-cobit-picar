import RPi.GPIO as IO
import time



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


try:
    while 1:
        '''
        IO.output(dirPin, True)
        for x in range (100):
            p.ChangeDutyCycle(x)
            time.sleep(0.1)
        time.sleep(0.5)
        for x in range (100, 0, -1):
            p.ChangeDutyCycle(x)
            time.sleep(0.1)
        time.sleep(0.5)
        IO.output(dirPin, False)   
        for x in range (100):
            p.ChangeDutyCycle(x)
            time.sleep(0.1)
        time.sleep(0.5)
        for x in range (100, 0, -1):
            p.ChangeDutyCycle(x)
            time.sleep(0.1)
        '''
       
        p.ChangeDutyCycle(100)
        IO.output(dirPin, False) 
        p2.ChangeDutyCycle(100)
        IO.output(dirPin2, False) 
        time.sleep(2)
except KeyboardInterrupt:
     IO.output(dirPin, False) 
     IO.output(pwmPin, False)
     IO.output(dirPin2, False) 
     IO.output(pwmPin2, False)
     IO.cleanup() 
