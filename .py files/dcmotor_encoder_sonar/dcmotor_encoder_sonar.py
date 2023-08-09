import RPi.GPIO as GPIO
import time

pi = 3.141592

GPIO.setmode(GPIO.BCM)

# sonar
TRIGGER_PIN = 17
ECHO_PIN = 27

class Sonar:
    def __init__(self, trigger_pin, echo_pin):
        self._trigger_pin = trigger_pin
        self._echo_pin = echo_pin

        self.blocked = False

        GPIO.setup(self._trigger_pin, GPIO.OUT)
        GPIO.setup(self._echo_pin, GPIO.IN)

        GPIO.output(self._trigger_pin, False)
    
    def check(self):
        GPIO.output(self._trigger_pin, True)
        time.sleep(0.00001) #10us
        GPIO.output(self._trigger_pin, False)

        while GPIO.input(self._echo_pin) == 0:
            pulse_start = time.time()
        while GPIO.input(self._echo_pin) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance+0, 2)

        return distance
    
sonar0 = Sonar(TRIGGER_PIN, ECHO_PIN)

# dcmotor
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

gear_ratio = 100.
CPR = 12.
wheel_DIA = 43. #mm
intermotor_DIST = 155. #mm

ticks_per_degree = gear_ratio*CPR/360
ticks_per_meter = gear_ratio*CPR/(wheel_DIA/1000)/pi

rotation_dc = 75
moving_dc = 75

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
        self.pwm.start(0)

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

    def do_encoderA(self, channel):
        self.encoder_ticks += 1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else -1

    def do_encoderB(self, channel):
        self.encoder_ticks += -1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else 1


dcmotor1 = DCMotor(ENCODER_1A_PIN, ENCODER_1B_PIN,
                   MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN)
dcmotor2 = DCMotor(ENCODER_2A_PIN, ENCODER_2B_PIN,
                   MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN)

# loop
try:
    while True:
        # sonar check and apply to sonar0.blocked
        if sonar0.blocked == False and sonar0.check() < 20:
            sonar0.blocked = True
            target_rotation = 90
            left_ticks_rotation_start = dcmotor1.encoder_ticks


        # rotation
        if sonar0.blocked == True:
            #turn right
            if target_rotation > 0:
                dcmotor1.control(1, rotation_dc)
                dcmotor2.control(2, rotation_dc)
            #turn left
            else:
                dcmotor1.control(2, rotation_dc)
                dcmotor2.control(1, rotation_dc)
            
            # check if rotation is finished
            left_ticks_rotation_progressed = dcmotor1.encoder_ticks - left_ticks_rotation_start
            progressed_rotation = ((left_ticks_rotation_progressed/ticks_per_meter) / ((intermotor_DIST/1000)/2))/pi*180
            if progressed_rotation >= target_rotation > 0:
                sonar0.blocked = False
            elif progressed_rotation <= target_rotation < 0:
                sonar0.blocked = False

        # moving forward
        else:
            dcmotor1.control(1, moving_dc)
            dcmotor2.control(1, moving_dc)

        print(dcmotor1.encoder_ticks, dcmotor2.encoder_ticks)
        print(sonar0.check())

except KeyboardInterrupt:
    GPIO.cleanup()