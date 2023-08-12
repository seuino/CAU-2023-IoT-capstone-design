#!/usr/bin/env python

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

        self._MAX_DIST = 20 # cm
        self._SPEED_OF_SOUND = 346 # 331 + 0.6*25 = 346m/s
        self._MAX_ECHO_TIME = self._MAX_DIST*1e-2*2 / self._SPEED_OF_SOUND
        self._MAX_PING_DELAY = 18000 # us
        self._max_time
        
        self.distance

        GPIO.setup(self._trigger_pin, GPIO.OUT)
        GPIO.setup(self._echo_pin, GPIO.IN)

        GPIO.output(self._trigger_pin, False)
    
    def trigger(self):
        ##################################################
        ##                   CAUTION                    ##
        ##################################################
        GPIO.output(self._trigger_pin, False)
        time.sleep(0.000004) # 4us
        GPIO.output(self._trigger_pin, True)
        time.sleep(0.00001) # 10us
        GPIO.output(self._trigger_pin, False)

        self._max_time = time.time() + self._MAX_PING_DELAY*1e-6
        while GPIO.input(self._echo_pin) == 0 and time.time() <= self._max_time:
            pass
        while GPIO.input(self._echo_pin) == 1:
            if time.time() > self._max_time:
                return False
        self._max_time = time.time() + self._MAX_ECHO_TIME
        return True
    
    def check(self):
        if self.trigger() == False:
            self.distance = self._MAX_DIST # echo pin is not cleared
        while GPIO.input(self._echo_pin) == 0: # check echo pulse width
            if time.time() > self._max_time:
                self.distance = self._MAX_DIST # beyond maximum distance
        self.distance = (time.time() - (self._max_time - self._MAX_ECHO_TIME) - 0.000005) \
                        * self._SPEED_OF_SOUND / 2 * 1e2 # cm

sonar = Sonar(TRIGGER_PIN, ECHO_PIN)


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

        self._KP = 2.5
        self._KI = 0.5
        self._KD = 0.01
        self._acc_error = 0

        self.encoder_ticks = 0

        self.recorded_ticks = [0,0] # 2 recent ~ old
        self.recorded_angular_vel = [0,0]

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
        self.recorded_ticks.insert(0, current_encoder_ticks)
        self.recorded_ticks.pop()
        self.recorded_angular_vel.insert(0, (self.record_ticks[0] - self.record_ticks[-1]) \
                                             / self._CPR / self._GEAR_RATIO * 2*pi \
                                             / event.last_duration)
        self.recorded_angular_vel.pop()

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
        self.pwm.ChangeDutyCycle(dc)
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
    
    def control_angular_vel(self, target_angular_vel):
        error = target_angular_vel - self.recorded_angular_vel[0]
        pre_error = target_angular_vel - self.recorded_angular_vel[1]
        self._acc_error += error

        dc_p = self._KP * error
        dc_i = self._KI * self._acc_error
        dc_d = self._KD * pre_error
        dc_pid = dc_p + dc_i + dc_d

        dc_pid = max(100, dc_pid) # max 100
        self.control_dc(dc_pid)

left_dcmotor = DCMotor(ENCODER_1A_PIN, ENCODER_1B_PIN,
                       MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN)
right_dcmotor = DCMotor(ENCODER_2A_PIN, ENCODER_2B_PIN,
                        MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN)

# rospy
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

left_ticks_msg = Int32()
right_ticks_msg = Int32()

rospy.initnode("raspi4", anonymous=False, disable_signals=True)

rate = rospy.Rate(15)

rospy.on_shutdown(GPIO.cleanup())

left_ticks_publisher = rospy.Publisher(name="left_ticks_pub", data_class=Int32,
                                       queue_size=1)
right_ticks_publisher = rospy.Publisher(name="right_ticks_pub", data_class=Int32,
                                        queue_size=1)


class TwoWheeledMobile:
    def __init__(self, *dcmotors, sonar):
        self._INTERMOTOR_DIST = 115. # mm
        self._WHEEL_DIA = 43. # mm

        self.linear_x # m/s
        # self.linear_y
        # self.linear_z
        # self.angular_x
        # self.angular_y
        self.angular_z # rad/s (counter-clockwise)

        self._left_target_linear_vel = 0 # m/s
        self._right_target_linear_vel = 0
        self._left_target_angular_vel = 0 # rad/s
        self._right_target_angular_vel = 0

        self.left_dcmotor = dcmotors[0]
        self.right_dcmotor = dcmotors[1]

        self.sonar = sonar

    def get_cmd_vel(self, data):
        self.linear_x = data.linear.x # m/s
        # self.linear_y = data.linear.y
        # self.linear_z = data.linear.z
        # self.angular_x = data.angular.x
        # self.angular_y = data.angular.y
        self.angular_z = data.angular.z # rad/s

        self._left_target_linear_vel = self.linear_x \
            - (self.angular_z * self._INTERMOTOR_DIST*1e-3/2)
        self._right_target_linear_vel = self.linear_x \
            + (self.angular_z * self._INTERMOTOR_DIST*1e-3/2)
        
        self._left_target_angular_vel = self._left_target_linear_vel / (self._WHEEL_DIA*1e-3/2)
        self._right_target_angular_vel = self._right_target_linear_vel / (self._WHEEL_DIA*1e-3/2)

    def achieve_cmd_vel(self):
        self.left_dcmotor.control_angular_vel(self._left_target_angular_vel)
        self.right_dcmotor.control_angular_vel(self._right_target_angular_vel)

robotic_vacuum = TwoWheeledMobile((left_dcmotor, right_dcmotor), sonar)

rospy.Subscriber("/cmd_vel", Twist, robotic_vacuum.get_cmd_vel)

rospy.Timer(rospy.Duration(0.1), robotic_vacuum.left_dcmotor.record_ticks)
rospy.Timer(rospy.Duration(0.1), robotic_vacuum.right_dcmotor.record_ticks)

rospy.Timer(rospy.Duration(0.1), robotic_vacuum.sonar.check) # minimum 60ms cycle


# loop
while not rospy.is_shutdown():
    try:
        robotic_vacuum.achieve_cmd_vel()

        # publish topics
        left_ticks_msg.data = left_dcmotor.encoder_ticks
        right_ticks_msg.data = right_dcmotor.encoder_ticks
        rospy.loginfo(left_ticks_msg)
        rospy.loginfo(right_ticks_msg)
        left_ticks_publisher.publish(left_ticks_msg)
        right_ticks_publisher.publish(right_ticks_msg)


        ##################################################
        ##                   CAUTION                    ##
        ##################################################
        rate.sleep()
    except KeyboardInterrupt:
        pass
