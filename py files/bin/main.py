#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

from smbus2 import SMBus
from mlx90614 import MLX90614

from parts.sonar import Sonar
from parts.dcmotor import DCMotor
from assembly.two_wheeled_moblie import TwoWheeledMobile
from config.pinout import pins

GPIO.setmode(GPIO.BCM)


# sensors
# Initialize I2C bus and MLX90614 sensor
bus = SMBus(1)
sensor = MLX90614(bus, address=0x5A)

# Define the temperature threshold (30 degrees Celsius)
temperature_threshold = 40


# sonar
sonar = Sonar(pins['TRIGGER_PIN'], pins['ECHO_PIN'])


# dcmotor
left_dcmotor = DCMotor(pins['ENCODER_1A_PIN'], pins['ENCODER_1B_PIN'],
                       pins['MOTOR_1A_PIN'], pins['MOTOR_1B_PIN'],
                       pins['ENABLE_1_PIN']
                       )
right_dcmotor = DCMotor(pins['ENCODER_2A_PIN'], pins['ENCODER_2B_PIN'],
                        pins['MOTOR_2A_PIN'], pins['MOTOR_2B_PIN'],
                        pins['ENABLE_2_PIN']
                        )


# rospy
import rospy
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

left_ticks_msg = Int32()
right_ticks_msg = Int32()

duration_record_ticks = 50 # ms

rospy.init_node("raspi4", anonymous=False, disable_signals=True)

rate = rospy.Rate(15)

# rospy.on_shutdown(GPIO.cleanup)

left_ticks_publisher = rospy.Publisher(name="left_ticks",
                                       data_class=Int32,
                                       queue_size=1,
                                       )
right_ticks_publisher = rospy.Publisher(name="right_ticks",
                                        data_class=Int32,
                                        queue_size=1,
                                        )
temperature_pub = rospy.Publisher('temperature_data',
                                  data_class=Float32,
                                  queue_size=10,
                                  )


# two_wheeled_mobile
robotic_vacuum = TwoWheeledMobile(sonar, left_dcmotor, right_dcmotor)

rospy.Subscriber("/cmd_vel", Twist, robotic_vacuum.get_cmd_vel)

rospy.Timer(rospy.Duration(duration_record_ticks*1e-3), robotic_vacuum.left_dcmotor.record_ticks)
rospy.Timer(rospy.Duration(duration_record_ticks*1e-3), robotic_vacuum.right_dcmotor.record_ticks)

# rospy.Timer(rospy.Duration(0.1), robotic_vacuum.sonar.check) # minimum 60ms cycle


while not rospy.is_shutdown():
    try:
        ##################################################
        ##                 Read sensors                 ##
        ##################################################
        # Read temperature
        ambient_temp = sensor.get_ambient()
        object_temp = sensor.get_object_1()

        # Check if the temperature is below the threshold
        if object_temp > temperature_threshold:
            # Publish the message for test
            print("emergency")

        ##################################################
        ##                Control Motors                ##
        ##################################################
        robotic_vacuum.achieve_cmd_vel()
        # For debug PID
        # print(left_dcmotor.recorded_angular_vel[0], robotic_vacuum._left_target_angular_vel)

        ##################################################
        ##                Publish topics                ##
        ##################################################
        left_ticks_msg.data = left_dcmotor.encoder_ticks
        right_ticks_msg.data = right_dcmotor.encoder_ticks
        left_ticks_publisher.publish(left_ticks_msg)
        right_ticks_publisher.publish(right_ticks_msg)
        # rospy.loginfo(left_ticks_msg)
        # rospy.loginfo(right_ticks_msg)
        temperature_pub.publish(object_temp)
        
        rate.sleep()
    
    except KeyboardInterrupt:
        GPIO.cleanup()
        break
