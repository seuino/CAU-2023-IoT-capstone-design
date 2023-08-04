import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIGGER_PIN = 17
ECHO_PIN = 27

GPIO.setup(TRIGGER_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.output(TRIGGER_PIN, False)

try:
    while True:
       GPIO.output(TRIGGER_PIN, True)
       time.sleep(0.00001) #10us
       GPIO.output(TRIGGER_PIN, False)

       while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

       while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

       pulse_duration = pulse_end - pulse_start

       distance = pulse_duration * 17150

       distance = round(distance+0, 2)
  
       print(distance)

       time.sleep(2)

except KeyboardInterrupt:
     GPIO.cleanup()