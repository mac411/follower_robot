# import the necessary packages
import numpy as np
import cv2
import array
import math
import matplotlib.pyplot as plt

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

# open webcam video stream
cap = cv2.VideoCapture(0)

# the output will be written to output.avi
out = cv2.VideoWriter(
    'output.avi',
    cv2.VideoWriter_fourcc(*'MJPG'),
    15.,
    (640,480))
# curve to get angular_velocity
c1 = 160
c2 = 480
# angular velocity = rate of change of the angle dw/dt
ang_max = math.pi/4
ang_min = -math.pi/4
y1 = ang_min*np.ones(c1)
y2 = np.linspace(ang_min, ang_max, c2-c1)
y3 = ang_max*np.ones(640-c2)
y = np.append(y1,y2)
ang_indx = np.append(y,y3)
x = np.arange(0, 640, 1)
#plt.plot(x, ang_indx)
#plt.xlabel('box center')
#plt.ylabel('angular velocity')
#plt.show()
angular_velocity = 0
avt = [0]
avts = [0]
current_state = 0
while(True):
    # Capture frame-by-frame
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
        cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (255, 0, 0), 2)
        xD = xB-xA
        xC = int(xA+xD/2)
        # print(xC)
        #if xC > 0:
            #cv2.line(frame, (xC,0), (xC,480), (0,255,0), 4)
        xweight = xweight + weight[i]*xC
        i = i+1
    print('weghted center', int(xweight))
    if len(boxes) > 0 :
        current_state = 1 #locked
        cv2.line(frame, (int(xweight),0), (int(xweight),480), (0,0,255), 4)
        angular_velocity = ang_indx[int(xweight)]
    else:
        current_state = 0 #idle

    avt = np.append(avt, angular_velocity)
    avs = smooth(avt, 25)
    avts = np.append(avts, 2*avs[len(avs)-1])
    #print('ang_vel', angular_velocity)
    #print('ang_vel_smooth', avts)
    #cv2.line(frame, (640,0), (640,480), (0,255,0),6)

    # Write the output video
    out.write(frame.astype('uint8'))
    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
# and release the output
out.release()
# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)


f = np.arange(1, len(avt)+1,1)
#yhat = smooth(angular_velocity_track,30)
plt.plot(f, avt)
plt.plot(f,avts,color = 'green')
plt.xlabel('cycle number')
plt.ylabel('angular velocity')
plt.show()
