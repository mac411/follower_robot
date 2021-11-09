import math
import numpy as np
import random

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

def motion_model():
    angular_velocity = 0    #no angular velocity until person detected
    prev_pos = [0, 0, 0]      #initial position 0 0 and make theta = 0
    lin_vel = 1
    n = 100
    obj_pos = [2, 0]
    sensor_data = [1, 0]
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
    k = 1000
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

next_pos = motion_loop()
print(next_pos)
