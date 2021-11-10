import cv2
import time
from datetime import datetime

cap0 = cv2.VideoCapture(0 + cv2.CAP_DSHOW)
ret0, frame0 = cap0.read()

first0 = None
statusList0 = [None, None]
detected0 = 0

timeStamp0 = []
time_prev = time.time()
count = 0
i = 0
while(1):
    #cX = 0 #try to find center of object (not working very well)
    #if statement resets the base frame
    #i = i+1
    #if i == 100:
    #    i = 0
    #    ret0, frame0 = cap0.read()
    #    print('base frame updated')
    #    first0 = None
    #    statusList0 = [None, None]
    #    detected0 = 0
    frame1 = cv2.flip(frame0, 0)

    #detect motion on camera 0
    check, color0 = cap0.read()
    status0 = 0
    gray0 = cv2.cvtColor(color0, cv2.COLOR_BGR2GRAY )
    gray0 = cv2.GaussianBlur(gray0,(21,21), 0)
    if first0 is None:
        first0 = gray0
        continue
    delta0 = cv2.absdiff(first0, gray0)
    threshold0 = cv2.threshold(delta0, 30, 255, cv2.THRESH_BINARY)[1]
    (contours0,_)=cv2.findContours(threshold0,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #j = 1
    #cX = 0
    for contour in contours0:
        if cv2.contourArea(contour) < 5000:
            continue
        #cX = int(contour[0,0,0] +cX)
        status0 = 1
        #j = j+1
    #cX = cX/j
    #print('center',cX)
    statusList0.append(status0)
    if statusList0[-1]==1 and statusList0[-2]==0:
        print('Object Detected')
        detected0 = 1
    if statusList0[-1]==0 and statusList0[-2]==1:
        print('Object Left Screen')
        detected0 = 0


    if ret0:
        #cv2.drawContours(threshold0, contours0, -1, (0,255,0), 3)
        #cv2.line(threshold0, (int(cX),0), (int(cX),480), (0,0,0), 4)
        cv2.imshow('Camera 0',threshold0)
    ret0, frame0 = cap0.read()

    k = cv2.waitKey(1)
    if k%256 == 27: #hit escape key to close cameras
        break

cap0.release()

cv2.destroyAllWindows()
