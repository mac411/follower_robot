#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import array
import math
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import uint8

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

#cv2.startWindowThread()

# open webcam video stream
#cap = cv2.VideoCapture(0)
cap  = None                          #STREAM OF FRAMES
# the output will be written to output.avi
out = cv2.VideoWriter(
    'output.avi',
    cv2.VideoWriter_fourcc(*'MJPG'),
    15.,
    (640,480))

center_pixel = 320
current_state = 0

def processor_callback(data):
    # Capture frame-by-frame
    cap = data
    ret, frame = cap.read()

    # resizing for faster detection
    frame = cv2.resize(frame, (640, 480))
    # using a greyscale picture, also for faster detection
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )
    biggest_box_index = 0
    print('all boxes', boxes)
    print('length', len(boxes))
    size = []
    weight = []
    for (x, y, w, h) in boxes:
        size.append(w*h)
    #biggest_box_index = size.index(max(size))
    total_area = sum(size)
    weight = [i / total_area for i in size]
    #print('weight',weight)

    avg_center = 0
    xweight = 0
    i = 0
    for (xA, yA, w, h) in boxes:
        xB = xA+w
        yB = yA+h
        # display the detected boxes in the colour picture
        #cv2.rectangle(frame, (xA, yA), (xB, yB),
                          #(255, 0, 0), 2)
        xD = xB-xA
        xC = int(xA+xD/2)
        #if xC > 0:
            #cv2.line(frame, (xC,0), (xC,480), (0,255,0), 4)
        xweight = xweight + weight[i]*xC
        i = i+1
    #print('weghted center', int(xweight))
    if len(boxes) > 0 :
        current_state = 1 #locked
        center_pixel = int(xweight)
        #cv2.line(frame, (int(xweight),0), (int(xweight),480), (0,0,255), 4)
        $angular_velocity_not_smooth = ang_indx[int(xweight)]
    else:
        current_state = 0 #idle

    #avt = np.append(avt, angular_velocity_not_smooth)
    #avs = smooth(avt, 25)
    #angular_velocity = 2*avs[len(avs)-1]
    #avts = np.append(avts, angular_velocity)

    #print('ang_vel', angular_velocity)
    #print('ang_vel_smooth', avts)
    #cv2.line(frame, (640,0), (640,480), (0,255,0),6)

    # Write the output video
    # out.write(frame.astype('uint8'))
    # Display the resulting frame
    # cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    state_pub.publish(np.uint8(current_state)
    pixel_pub.publish(np.uint8(center_pixel))
# When everything done, release the capture
cap.release()
# and release the output
out.release()
# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)


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

def image_process():
    rospy.init_node('image_processor', anonymous=True)
    state_pub = rospy.Publisher('state', uint8, queue_size=10)
    pixel_pub = rospy.Publisher('pixel', uint8, queue_size=10)
    rospy.Subscriber('follower_robot/camera1/image_topic', Image, processor_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        image_process()
    except rospy.ROSInterruptException:
        pass
