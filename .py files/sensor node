import rospy
from sensor_msgs.msg import Temperature
import smbus

# MLX90614 I2C address
MLX90614_I2C_ADDR = 0x5A

# MLX90614 Register Addresses
TEMP_AMBIENT_REG = 0x06
TEMP_OBJECT_REG = 0x07

def read_temperature():
    bus = smbus.SMBus(1)
    temperature = bus.read_word_data(MLX90614_I2C_ADDR, TEMP_OBJECT_REG)
    temperature = (temperature * 0.02) - 273.15
    return temperature

def publish_temperature_threshold():
    rospy.init_node('mlx90614_threshold_publisher', anonymous=True)
    pub_temperature = rospy.Publisher('temperature', Temperature, queue_size=10)
    pub_threshold = rospy.Publisher('temperature_threshold', Temperature, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz

    while not rospy.is_shutdown():
        temperature_celsius = read_temperature()

        # Publish temperature
        temperature_msg = Temperature()
        temperature_msg.header.stamp = rospy.Time.now()
        temperature_msg.temperature = temperature_celsius
        pub_temperature.publish(temperature_msg)

        # Publish threshold status
        threshold_msg = Temperature()
        threshold_msg.header.stamp = rospy.Time.now()
        threshold_msg.temperature = 1 if temperature_celsius > 40.0 else 0
        pub_threshold.publish(threshold_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_temperature_threshold()
    except rospy.ROSInterruptException:
        pass
