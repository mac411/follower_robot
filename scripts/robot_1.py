#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)
def image_callback(self,data):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image Window", cv_image)

def motion_command_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    motion_command = data

def state_command_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    state_command = data

def robot():
    state_pub = rospy.Publisher('robot_state', String, queue_size=10)
    data_pub = rospy.Publisher('sensor_data', String, queue_size=10)
    rospy.init_node('robot', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber('motion_command', String, motion_command_callback)
        rospy.Subscriber('state_command', String, state_command_callback)
        __init__(self)
        rospy.init_node('image_converter', anonymous=True)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting Down')
        cv2.destroyAllWindows()
        try:
             self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
         except CvBridgeError as e:
              print(e)

        message = 'Test'
        rospy.loginfo(message)
        state_pub.publish(message)
        data_pub.publish(message)

        rate.sleep()

if __name__ == '__main__':
    try:
        robot()
    except rospy.ROSInterruptException:
        pass
