#!/usr/bin/python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cv_image = None
bridge = CvBridge()
state_pub = rospy.Publisher('robot_state', String, queue_size=10)
data_pub = rospy.Publisher('sensor_data', String, queue_size=10)

def image_callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(1)

def state_command_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    state_command = data

def robot():
    rospy.init_node('robot', anonymous=True)
    rospy.Subscriber('state_command', String, state_command_callback)
    rospy.Subscriber("follower_robot/camera1/image_topic",Image,image_callback)
    cv2.startWindowThread()

    rospy.spin()
    # cv2.destroyAllWindows() ?

if __name__ == '__main__':
    try:
        robot()
    except rospy.ROSInterruptException:
        pass
