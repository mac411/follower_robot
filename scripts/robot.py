#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def motion_command_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def state_command_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def robot():
    state_pub = rospy.Publisher('robot_state', String, queue_size=10)
    data_pub = rospy.Publisher('sensor_data', String, queue_size=10)
    rospy.init_node('robot', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        message = 'Test' 
        rospy.loginfo(message)
        state_pub.publish(message)
        data_pub.publish(message)
        rospy.Subscriber('motion_command', String, motion_command_callback)
        rospy.Subscriber('state_command', String, state_command_callback)
        rate.sleep()

if __name__ == '__main__':
    try:
        robot()
    except rospy.ROSInterruptException:
        pass