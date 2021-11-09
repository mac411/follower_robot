#!/usr/bin/env python
import rospy
import math
import cv2
import numpy as np
import random
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

angular_velocity = 0    #no angular velocity until person detected
prev_pos = [0, 0, 0]      #initial position 0 0 and make theta = 0
lin_vel = 0
robot_state = 0
sensor_data = 0

def pixel_2_ang_vel(xweight):
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
    angular_velocity = ang_indx[int(xweight)]
    return angular_velocity
def dist_2_lin_vel(distance):
    max_distance = 20
    c1 = 2
    c2 = 15
    lin_max = 2
    #x_index = np.arange(0, max_distance, 1)
    y1 = np.zeros(c1)
    y2 = np.linspace(0, lin_max, c2-c1)
    y3 = lin_max*np.ones(max_distance-c2)
    y = np.append(y1,y2)
    y = np.append(y,y3)
    lin_vel = y[int(distance)]
    return lin_vel

def velocity_model(prev_pos, lin_vel, ang_vel, n):
    a = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]
    dt = 1 #how long we want the loop to take
    x_prev = prev_pos[0]
    y_prev = prev_pos[1]
    theta_prev = prev_pos[2]
    v = lin_vel
    w = ang_vel
    if w == 0:
        w = 0.000001
    err1 = np.random.randn(1,n)*(a[0]*v**2+a[1]*w**2)
    err2 = np.random.randn(1,n)*(a[2]*v**2+a[3]*w**2)
    err3 = np.random.randn(1,n)*(a[4]*v**2+a[5]*w**2)
    vh = v + err1
    wh = w + err2
    rh = vh/wh
    gamma = err3
    x_next = x_prev - rh*math.sin(theta_prev) + rh*np.sin(theta_prev+wh*dt)
    y_next = y_prev + rh*math.cos(theta_prev) - rh*np.cos(theta_prev+wh*dt)
    theta_next = theta_prev + wh*dt + gamma*dt
    return x_next, y_next, theta_next
def normpdf(x, mean, sd):
    var = float(sd)**2
    denom = (2*math.pi*var)**.5
    num = math.exp(-(float(x)-float(mean))**2/(2*var))
    return num/denom
def landmark(current_pos, obj_pos, sensor_data):
    #current_pos & obj_pos =[x,y,th]; sensot_data = [distance, bearing]
    r = math.sqrt((obj_pos[0]-current_pos[0])**2 + (obj_pos[1]-current_pos[1])**2)
    phi = math.atan2(obj_pos[1]-current_pos[1],obj_pos[0]-current_pos[0])
    m1 = sensor_data[1]-phi
    m2 = abs(m1+2*math.pi)
    m3 = abs(m1-2*math.pi)
    mean_phi = min(abs(m1), m2, m3)
    q = normpdf(sensor_data[0]-r,0, 0.1) * normpdf(mean_phi,0, 0.09) #zero mean gausian distribution
    return q

def motion_model(prev_pos, lin_vel, angular_velocity,obj_pos,sensor_data,state):
    #just spin if not locked on
    if state == 0:
        lin_vel = 0
        angular_velocity = 0.1*math.pi
        velocity_model(prev_pos, lin_vel, angular_velocity, 1)
        next_pos = [x, y, t]
        return next_pos
        break
    else:
        n = 1000
        #obj_pos = [2, 0]
        #sensor_data = [1, 0]
        # prev_pos from most_likely_next_pos
        # lin_vel from ultrasound sensor_data
        #angular_velocity from sensor_data
        x, y, t = velocity_model(prev_pos, lin_vel, angular_velocity, n)
        q = []
        for i in range(0,n):
            next_pos = [x[0,i], y[0,i], t[0,i]]
            # object_pos from gazebo ("human") x,y
            # sensor_data = ultrasound data
            q_new = landmark(next_pos, obj_pos, sensor_data)
            q.append(q_new)
        idx = range(0,n)
        k = 100
        rwi = random.choices(idx,weights = q, k = k)
        sx = 0
        sy = 0
        st = 0
        for i in range(0,k):
            samp_pos = x[0,rwi[i]], y[0,rwi[i]], t[0,rwi[i]]
            sx = samp_pos[0]+sx
            sy = samp_pos[1]+sy
            st = samp_pos[2]+st
        x_a = sx/k
        y_a = sy/k
        t_a = st/k
        nex_pos = [x_a, y_a, t_a]
        print('next_pos', next_pos) #position for simulator and prev_position
        return next_pos

def robot_state_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    robot_state = data

def sensor_data_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    sensor_data = data

def controller():
    motion_pub = rospy.Publisher('motion_command', String, queue_size=10)
    state_pub = rospy.Publisher('state_command', String, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #Retrive robot state and sesor data from topics
        rospy.Subscriber('robot_state', String, robot_state_callback)
        rospy.Subscriber('sensor_data', String, sensor_data_callback)
        #pixel to angular velocity
        angular_velocity = pixel_2_ang_vel(sensor_data[1])
        #distance to linear velocity
        lin_velocity = dist_2_lin_vel(sensor_data[0])
        #get motion command
        next_pos = motion_model(prev_pos, lin_vel, angular_velocity, obj_pos, sensor_data, robot_state)
        motion_pub.publish(next_pos)

        message = 'Test'
        rospy.loginfo(message)
        motion_pub.publish(message)
        state_pub.publish(message)  #WHAT IS STATE PUBLISH??? state of controller?
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
