#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def robot_state_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def sensor_data_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def controller():
    motion_pub = rospy.Publisher('motion_command', String, queue_size=10)
    state_pub = rospy.Publisher('state_command', String, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = 'Test' 
        rospy.loginfo(message)
        motion_pub.publish(message)
        state_pub.publish(message)
        rospy.Subscriber('robot_state', String, robot_state_callback)
        rospy.Subscriber('sensor_data', String, sensor_data_callback)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass