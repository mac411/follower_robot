#!/usr/bin/python
import rospy
import math
import numpy as np
import random
from std_msgs.msg import String
from std_msgs.msg import Int64
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

class robot_controller:

    def __init__(self):
        self.status_sub = rospy.Subscriber('detection_status', Int64, self.robot_status_callback)
        self.detection_status = None
        self.location_sub = rospy.Subscriber('object_location', Int64, self.location_callback)
        self.object_location = None
        self.orientation_sub = rospy.Subscriber('object_orientation', Int64, self.orientation_callback)
        self.object_orientation = None
        self.motion_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.motion_command = ModelState('follower_robot',None,None,'base')
        self.ang_vel = None
        self.lin_vel = None
        self.prev_pos = [0, 0, 0]
        self.next_pos = [0, 0, 0]

    def robot_status_callback(self,data):
        self.detection_status = data.data

    def location_callback(self,data):
        self.object_location = data.data

    def orientation_callback(self,data):
        self.object_orientation = data.data

    def update_lin_vel(self):
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
        self.lin_vel = y[int(self.object_location)]

    def update_ang_vel(self):
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
        self.ang_vel = ang_indx[int(self.object_orientation)]

    def velocity_model(self,n):
        a = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]
        dt = 1 #how long we want the loop to take
        x_prev = self.prev_pos[0]
        y_prev = self.prev_pos[1]
        theta_prev = self.prev_pos[2]
        v = self.lin_vel
        w = self.ang_vel
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

    def landmark_model(self, current_pos, obj_pos, sensor_data):
        #current_pos & obj_pos =[x,y,th]; sensot_data = [distance, bearing]
        r = math.sqrt((obj_pos[0]-current_pos[0])**2 + (obj_pos[1]-current_pos[1])**2)
        phi = math.atan2(obj_pos[1]-current_pos[1],obj_pos[0]-current_pos[0])
        m1 = sensor_data[1]-phi
        m2 = abs(m1+2*math.pi)
        m3 = abs(m1-2*math.pi)
        mean_phi = min(abs(m1), m2, m3)
        q = self.normpdf(sensor_data[0]-r, 0, 0.1) * self.normpdf(mean_phi, 0, 0.09) #zero mean gausian distribution
        return q

    def normpdf(self, x, mean, sd):
        var = float(sd)**2
        denom = (2*math.pi*var)**.5
        num = math.exp(-(float(x)-float(mean))**2/(2*var))
        return num/denom

    def motion_model(self, obj_pos, sensor_data):
        #just spin if not locked on
        if self.detection_status == 0:
            self.lin_vel = 0
            self.ang_vel = 0.1*math.pi
            x, y, t = self.velocity_model(1)
            self.next_pos = [x, y, t]
        else:
            n = 1000
            #obj_pos = [2, 0]
            #sensor_data = [1, 0]
            # prev_pos from most_likely_next_pos
            # lin_vel from ultrasound sensor_data
            #angular_velocity from sensor_data
            x, y, t = self.velocity_model(n)
            q = []
            for i in range(0,n):
                next = [x[0,i], y[0,i], t[0,i]]
                # object_pos from gazebo ("human") x,y
                # sensor_data = ultrasound data
                q_new = self.landmark(next, obj_pos, sensor_data)
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
            self.prev_pos = self.next_pos
            self.next_pos = [x_a, y_a, t_a]
            # print('next_pos', next_pos) #position for simulator and prev_position

    def publish_command(self):
        self.update_ang_vel()
        self.update_lin_vel()
        self.motion_command.twist.linear.x = self.lin_vel
        self.motion_command.twist.angular.z = self.ang_vel
        self.motion_pub.publish(self.motion_command)

def control():
    rospy.init_node('controller', anonymous=True)
    rc = robot_controller()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rc.publish_command()
        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
