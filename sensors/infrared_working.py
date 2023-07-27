import RPi.GPIO as GPIO
import time

sensor = 16
buzzer = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor,GPIO.IN)
GPIO.setup(buzzer,GPIO.OUT)

GPIO.output(buzzer,False)
print "IR Sensor Ready....."
print " "

try: 
   while True:
      if GPIO.input(sensor):
          msg='Now come close to the temperature sensor'
          return msg,True


except KeyboardInterrupt:
    GPIO.cleanup()
