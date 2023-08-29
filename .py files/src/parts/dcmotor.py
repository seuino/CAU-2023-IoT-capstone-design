import RPi.GPIO as GPIO
from collections import deque


class DCMotor:
    def __init__(self, encoderA_pin, encoderB_pin,
                 motorA_pin, motorB_pin, enable_pin):
        self._encoderA_pin = encoderA_pin
        self._encoderB_pin = encoderB_pin
        self._motorA_pin = motorA_pin
        self._motorB_pin = motorB_pin
        self._enable_pin = enable_pin

        self._CPR = 12.
        self._GEAR_RATIO = 100.

        self._KP = 3
        self._KI = 0
        self._KD = 0
        self._acc_error = 0

        self.encoder_ticks = 0

        self._is_first_record = True
        self.recorded_ticks = deque([0,0], maxlen=2) # 2 old ~ recent
        self.recorded_angular_vel = deque([0,0], maxlen=2)

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

    def do_encoderA(self, channel):
        self.encoder_ticks += 1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else -1

    def do_encoderB(self, channel):
        self.encoder_ticks += -1 if GPIO.input(self._encoderA_pin) == GPIO.input(self._encoderB_pin) else 1

    def record_ticks(self, event):
        current_encoder_ticks = self.encoder_ticks
        self.recorded_ticks.append(current_encoder_ticks)

        ##################################################
        ##                   CAUTION                    ##
        ##################################################
        if self._is_first_record is True:
            delta_t = 0.1
            self._is_first_record = False
        else:
            delta_t = event.last_duration
        # delta_t = 0.1

        self.recorded_angular_vel.append((self.recorded_ticks[-1] - self.recorded_ticks[-2]) \
                                         / self._CPR / self._GEAR_RATIO * 2*pi \
                                         / delta_t)

    # def control_dc(self, type, dc):
    #     self.pwm.ChangeDutyCycle(dc)
    #     if type == 1:
    #         GPIO.output(self._motorA_pin, True)
    #         GPIO.output(self._motorB_pin, False)
    #     elif type == 2:
    #         GPIO.output(self._motorA_pin, False)
    #         GPIO.output(self._motorB_pin, True)
    #     elif type == 3:
    #         GPIO.output(self._motorA_pin, True)
    #         GPIO.output(self._motorB_pin, True)
    def control_dc(self, dc):
        self.pwm.ChangeDutyCycle(abs(dc))
        type = 1 if dc > 0 else 2
        if type == 1:
            GPIO.output(self._motorA_pin, True)
            GPIO.output(self._motorB_pin, False)
        elif type == 2:
            GPIO.output(self._motorA_pin, False)
            GPIO.output(self._motorB_pin, True)
        elif type == 3:
            GPIO.output(self._motorA_pin, True)
            GPIO.output(self._motorB_pin, True)
    
    def control_pid(self, target_angular_vel):
        ##################################################
        ##                   CAUTION                    ##
        ##################################################
        # self.record_ticks()

        error = target_angular_vel - self.recorded_angular_vel[-1]
        pre_error = target_angular_vel - self.recorded_angular_vel[-2]
        self._acc_error += error

        dc_p = self._KP * error
        dc_i = self._KI * self._acc_error
        dc_d = self._KD * pre_error
        dc_pid = dc_p + dc_i + dc_d

        if dc_pid > 0:
            dc_pid = min(100, dc_pid) # max 100
        else:
            dc_pid = max(-100, dc_pid) # min -100        
        self.control_dc(dc_pid)