#!/usr/bin/python
import rospy
import cv2
import numpy as np
import array
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int64

# def smooth(y, box_pts):
#     box = np.ones(box_pts)/box_pts
#     y_smooth = np.convolve(y, box, mode='same')
#     return y_smooth

# cv2.startWindowThread()
# the output will be written to output.avi
# out = cv2.VideoWriter(
#     'output.avi',
#     cv2.VideoWriter_fourcc(*'MJPG'),
#     15.,
#     (640, 480))

# # When everything done, release the capture
# cap.release()
# # and release the output
# out.release()
# # finally, close the window
# cv2.destroyAllWindows()
# cv2.waitKey(1)

class image_processor:

    def __init__(self):
        self.image_sub = rospy.Subscriber('follower_robot/camera1/image_topic', Image, self.processor_callback)
        self.status_pub = rospy.Publisher('detection_status', Int64, queue_size=10)
        self.orientation_pub = rospy.Publisher('object_orientation', Int64, queue_size=10)
        self.bridge = CvBridge()
        # initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.detection_status = Int64()
        self.object_orientation = Int64()


    def processor_callback(self, data):
        # Convert image to make it readable by OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Resizing for faster detection
        # self.cv_image = cv2.resize(self.cv_image, (640, 480))

        # Use grayscale picture, also for faster detection
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        # Detect people in the image
        # Returns the bounding boxes for the detected objects
        boxes, weights = self.hog.detectMultiScale(cv_image, winStride=(8, 8))
        # biggest_box_index = 0
        # print('all boxes', boxes)
        # print('length', len(boxes))
        size = []
        weight = []
        for (x, y, w, h) in boxes:
            size.append(w*h)
        # biggest_box_index = size.index(max(size))
        total_area = sum(size)
        weight = [i / total_area for i in size]
        # print('weight',weight)

        # avg_center = 0
        xweight = 0
        i = 0
        for (xA, yA, w, h) in boxes:
            xB = xA+w
            # yB = yA+h
            # display the detected boxes in the colour picture
            # cv2.rectangle(frame, (xA, yA), (xB, yB),
                            # (255, 0, 0), 2)
            xD = xB-xA
            xC = int(xA+xD/2)
            # if xC > 0:
                # cv2.line(frame, (xC,0), (xC,480), (0,255,0), 4)
            xweight = xweight + weight[i]*xC
            i = i+1
        # print('weighted center', int(xweight))
        if len(boxes) > 0:
            self.detection_status.data = int(1)  # locked
            self.object_orientation.data = int(xweight)
            # cv2.line(frame, (int(xweight),0), (int(xweight),480), (0,0,255), 4)
            # angular_velocity_not_smooth = ang_indx[int(xweight)]
        else:
            self.detection_status.data = int(0)  # idle

        # avt = np.append(avt, angular_velocity_not_smooth)
        # avs = smooth(avt, 25)
        # angular_velocity = 2*avs[len(avs)-1]
        # avts = np.append(avts, angular_velocity)

        # print('ang_vel', angular_velocity)
        # print('ang_vel_smooth', avts)
        # cv2.line(frame, (640,0), (640,480), (0,255,0),6)

        # Write the output video
        # out.write(frame.astype('uint8'))
        # Display the resulting frame
        # cv2.imshow('frame',frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

def image_process():
    rospy.init_node('image_processor', anonymous=True)
    ip = image_processor()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        ip.status_pub.publish(ip.detection_status)
        ip.orientation_pub.publish(ip.object_orientation)
        rate.sleep()

if __name__ == '__main__':
    try:
        image_process()
    except rospy.ROSInterruptException:
        pass
