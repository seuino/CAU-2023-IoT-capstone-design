#!/usr/bin/env python
# rospy
import rospy
import RPi.GPIO as GPIO
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
rospy.init_node("raspi4_talker", anonymous=False)
rate = rospy.Rate(20)
pi = 3.141592
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
GPIO.setmode(GPIO.BCM)


# sonar
TRIGGER_PIN = 17
ECHO_PIN = 27

# GPIO  clean up
def ros_shutdown():
    print("ros_shutdown")
    GPIO.cleanup()


rospy.on_shutdown(ros_shutdown)

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

class SubscribeAndPublish:
    
    def __init__(self):
        self.rotate_is=False
        self.left_ticks_rotation_start=0
        self.rotation_dc = 75
        self.moving_dc = 80
        self.target_rotate=0
        self.dcmotor1 = DCMotor(ENCODER_1A_PIN, ENCODER_1B_PIN,
                    MOTOR_1A_PIN, MOTOR_1B_PIN, ENABLE_1_PIN)
        self.dcmotor2 = DCMotor(ENCODER_2A_PIN, ENCODER_2B_PIN,
                    MOTOR_2A_PIN, MOTOR_2B_PIN, ENABLE_2_PIN)
        self.rotate_angle=0
        self.stop=False
        self.rotate=False
        self.left_ticks_msg = Int32()
        self.right_ticks_msg = Int32()
        self.left_ticks_publisher = rospy.Publisher(name="left_ticks", data_class=Int32,
                                            queue_size=1)
        self.right_ticks_publisher = rospy.Publisher(name="right_ticks", data_class=Int32,
                                                queue_size=1)
        
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback,(self.dcmotor1, self.dcmotor2))
        

    def callback(self, data, args):
        # print(data.ranges[360])
        x=data.linear.x
        z=data.angular.z
        dcmotor1=args[0]
        dcmotor2=args[1]
        self.rotate_is=True
        self.left_ticks_rotation_start = self.dcmotor1.encoder_ticks
        self.rotate_angle=self.teleop_key(x,z)
        self.target_rotate=self.rotate_angle
        
    
    def publish_encoder(self):
        self.left_ticks_msg.data = self.dcmotor1.encoder_ticks
        self.right_ticks_msg.data = self.dcmotor2.encoder_ticks

        self.right_ticks_publisher.publish(self.right_ticks_msg)
        self.left_ticks_publisher.publish(self.left_ticks_msg)

    def teleop_key(self,x,z):
        if x==0.5 and z==0:
            #moving foward/ not rotate
            self.stop=False
            self.rotate_is=False
            rotate_angle=0
            return rotate_angle
        elif x==0.5 and z==-1.0:
            #moving foward/ trun right 45
            self.stop=False
            rotate_angle=45
            return rotate_angle           
        elif x==0 and z==-1.0:
            #moving foward/ trun right 90
            self.stop=False
            rotate_angle=90
            return rotate_angle
        elif x==-0.5 and z==1.0:
            #moving foward/ trun right 135
            self.stop=False
            rotate_angle=135
            return rotate_angle
        elif x==-0.5 and z==0:
            #moving foward/ trun right 180
            self.stop=False
            rotate_angle=180
            return rotate_angle
        elif x==-0.5 and z==-1.0:
            #moving foward/ trun left 135
            self.stop=False
            rotate_angle=-135
            return rotate_angle
        elif x==0 and z==1.0:
            #moving foward/ trun left 90
            self.stop=False
            rotate_angle=-90
            return rotate_angle
        elif x==0.5 and z==1.0:
            #moving foward/ turn left 45
            self.stop=False
            rotate_angle=-45
            return rotate_angle
        elif x==0.5 and z==1.0:
            self.stop=True

def main():
    sub_pub=SubscribeAndPublish()
    temp_rotate=999

    while not rospy.is_shutdown():
        sub_pub.publish_encoder()
        # stop
        if sub_pub.stop:
            sub_pub.dcmotor1.control(3, sub_pub.rotation_dc)
            sub_pub.dcmotor1.control(3, sub_pub.rotation_dc)
        # if temp_rotate!=sub_pub.rotate_angle and sub_pub.rotate_angle!=0:
        #     sub_pub.rotate_is=True
        #     left_ticks_rotation_start = sub_pub.dcmotor1.encoder_ticks
        #     target_rotate=sub_pub.rotate_angle
        if sub_pub.rotate_is:
            #turn right
            if sub_pub.target_rotate > 0:
                sub_pub.dcmotor1.control(1, sub_pub.rotation_dc)
                sub_pub.dcmotor2.control(2, sub_pub.rotation_dc)
            #turn left
            else:
                sub_pub.dcmotor1.control(2, sub_pub.rotation_dc)
                sub_pub.dcmotor2.control(1, sub_pub.rotation_dc)
            
            # check if rotation is finished
            left_ticks_rotation_progressed = sub_pub.dcmotor1.encoder_ticks - sub_pub.left_ticks_rotation_start
            progressed_rotation = ((left_ticks_rotation_progressed/ticks_per_meter) / ((intermotor_DIST/1000)/2))/pi*180
            if progressed_rotation >= sub_pub.target_rotate > 0:
                sub_pub.rotate_is = False
                
            elif progressed_rotation <= sub_pub.target_rotate < 0:
                sub_pub.rotate_is = False
        else:
            sub_pub.dcmotor1.control(1, sub_pub.moving_dc)
            sub_pub.dcmotor2.control(1, sub_pub.moving_dc)
        temp_rotate=sub_pub.rotate_angle
        print(sub_pub.rotate_angle)
        rate.sleep()

if __name__ == "__main__":
    main()
    
