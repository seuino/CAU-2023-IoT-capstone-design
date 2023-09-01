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

GPIO.setup(ENCODER_1A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_1B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_1A_PIN, GPIO.OUT)
GPIO.setup(MOTOR_1B_PIN, GPIO.OUT)
GPIO.setup(ENABLE_1_PIN, GPIO.OUT)

GPIO.setup(ENCODER_2A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_2B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_2A_PIN, GPIO.OUT)
GPIO.setup(MOTOR_2B_PIN, GPIO.OUT)
GPIO.setup(ENABLE_2_PIN, GPIO.OUT)

class DCMotor:
    def __init__(self, motorA_pin, motorB_pin, enable_pin):
        self._motorA_pin = motorA_pin
        self._motorB_pin = motorB_pin
        self._enable_pin = enable_pin

        self.encoder_ticks = 0
        self.degree = 0

        self.pwm = GPIO.PWM(self._enable_pin, 1000)
        self.pwm.start

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

dcmotor1 = DCMotor(MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN)
dcmotor2 = DCMotor(MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN)

def do_encoder_1A(channel):
    dcmotor1.encoder_ticks += 1 if GPIO.input(ENCODER_1A_PIN) == GPIO.input(ENCODER_1B_PIN) else -1
def do_encoder_1B(channel):
    dcmotor1.encoder_ticks += -1 if GPIO.input(ENCODER_1A_PIN) == GPIO.input(ENCODER_1B_PIN) else 1
def do_encoder_2A(channel):
    dcmotor2.encoder_ticks += 1 if GPIO.input(ENCODER_2A_PIN) == GPIO.input(ENCODER_2B_PIN) else -1
def do_encoder_2B(channel):
    dcmotor2.encoder_ticks += -1 if GPIO.input(ENCODER_2A_PIN) == GPIO.input(ENCODER_2B_PIN) else 1

GPIO.add_event_detect(ENCODER_1A_PIN, GPIO.BOTH, callback=do_encoder_1A)
GPIO.add_event_detect(ENCODER_1B_PIN, GPIO.BOTH, callback=do_encoder_1B)
GPIO.add_event_detect(ENCODER_2A_PIN, GPIO.BOTH, callback=do_encoder_2A)
GPIO.add_event_detect(ENCODER_2B_PIN, GPIO.BOTH, callback=do_encoder_2B)

try:
    while True:
        dcmotor1.control(1, 75)
        dcmotor2.control(1, 75)
        print(dcmotor1.encoder_ticks, dcmotor2.encoder_ticks)

except KeyboardInterrupt:
    GPIO.cleanup()