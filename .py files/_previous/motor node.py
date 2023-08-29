import rospy
from sensor_msgs.msg import Temperature

def other_node_callback(msg):
    if msg.temperature == 1:
        rospy.loginfo("1이 수신되었습니다!")
        # 특정경로 이동 모터 제어
    else 

def other_node_subscriber():
    rospy.init_node('other_node', anonymous=True)
    rospy.Subscriber('temperature_threshold', Temperature, other_node_callback)
    rospy.spin()

if __name__ == '__main__':
    other_node_subscriber()
