import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

ENCODER_1A_PIN = 25
ENCODER_1B_PIN = 16
MOTOR_1A_PIN = 23
MOTOR_1B_PIN = 24
ENABLE_1_PIN = 12

ENCODER_2A_PIN = 22
ENCODER_2B_PIN = 26
MOTOR_2A_PIN = 5
MOTOR_2B_PIN = 6
ENABLE_2_PIN = 13

class DCMotor:
    def __init__(self, encoderA_pin, encoderB_pin,
                 motorA_pin, motorB_pin, enable_pin):
        
        self._encoderA_pin = encoderA_pin
        self._encoderB_pin = encoderB_pin
        self._motorA_pin = motorA_pin
        self._motorB_pin = motorB_pin
        self._enable_pin = enable_pin

        self.encoder_ticks = 0
        self.degree = 0

        GPIO.setup(self._encoderA_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._encoderB_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._motorA_pin, GPIO.OUT)
        GPIO.setup(self._motorB_pin, GPIO.OUT)
        GPIO.setup(self._enable_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self._enable_pin, 1000)

        GPIO.add_event_detect(self._encoderA_pin, GPIO.BOTH,
                              callback=self.do_encoderA)
        GPIO.add_event_detect(self._encoderB_pin, GPIO.BOTH,
                              callback=self.do_encoderB)

    def control(self, type, dc):
        self.pwm.ChangeDutyCycle(dc)
        if type == 1:
            GPIO.output(self._motorA_pin, True)
            GPIO.output(self._motorB_pin, False)
        elif type == 2:
            GPIO.output(self._motorA_pin, False)
            GPIO.output(self._motorB_pin, True)
        elif type == 3:
            GPIO.output(self._motorA_pin, True)
            GPIO.output(self._motorB_pin, True)

    def do_encoderA(self):
        self.encoder_ticks += 1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else -1

    def do_encoderB(self):
        self.encoder_ticks += -1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else 1


dcmotor1 = DCMotor(ENCODER_1A_PIN, ENCODER_1B_PIN,
                   MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN)
dcmotor2 = DCMotor(ENCODER_2A_PIN, ENCODER_2B_PIN,
                   MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN)

try:
    while True:
       dcmotor1.control(1, 100)
       dcmotor2.control(1, 100)
       print(dcmotor1.encoder_ticks, dcmotor2.encoder_ticks)

except KeyboardInterrupt:
     GPIO.cleanup()