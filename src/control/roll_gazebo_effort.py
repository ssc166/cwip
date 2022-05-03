#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
import pylab as pl
import control

# from sympy.physics.mechanics import *



def gazebo_setting():
    os.system('rosservice call /gazebo/set_physics_properties "{time_step: 0.001, max_update_rate: 1000.0, gravity: {x: 0.0, y: 0.0, z: -9.8}, ode_config: {auto_disable_bodies: False, sor_pgs_precon_iters: 0, sor_pgs_iters: 200, sor_pgs_w: 1.0, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2, max_contacts: 20}}"')
    # os.system('rosservice call gazebo/unpause_physics')
    rospy.loginfo("Simulation Start")
    
def roll_K_gain_R():
        
    K, S, E = control.lqr(A_R, B_R, Q, R)
    
    return K, S, E

def roll_K_gain_L():
        
    K, S, E = control.lqr(A_L, B_L, Q, R)
    
    return K, S, E

def rot_x(theta):
    rot_x = np.array([[1,0,0],
                      [0,np.cos(theta),-np.sin(theta)],
                      [0,np.sin(theta),np.cos(theta)]])
    return rot_x

def rot_y(theta):
    rot_y = np.array([[np.cos(theta),0,np.sin(theta)],
                       [0,1,0],
                       [-np.sin(theta),0,np.cos(theta)]])
    return rot_y

def rot_z(theta):
    rot_z = np.array([[np.cos(theta),-np.sin(theta),0],
                       [np.sin(theta),np.cos(theta),0],
                       [0,0,1]])
    return rot_z

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w
    
    ysqr = y * y
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)
    
    return X, Y, Z
        
    
def get_model_state(body_name):
    
    body_state = get_body(model_name = body_name)
    
    return body_state

def get_body_ori():
    global body_state
    
    body_state = get_model_state(body_name)
    
    X, Y, Z = quaternion_to_euler_angle(body_state.pose.orientation)
    body_ori_x = X
    body_ori_y = Y
    body_ori_z = Z
    
    return body_ori_x, body_ori_y, body_ori_z

def get_body_vel():
    global body_state
    
    body_state = get_model_state(body_name)
    
    body_vel_x = body_state.twist.angular.x
    body_vel_y = body_state.twist.angular.y
    body_vel_z = body_state.twist.angular.z
    
    return body_vel_x, body_vel_y, body_vel_z
    
def get_link_state(link_name_main, reference_frame):
    get_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    
    # reference_frame = gimbal_name
    # gimbal_state = get_state(link_name = gimbal_name)
    link_state = get_state(link_name= link_name_main, reference_frame = reference_frame)

    return link_state

def get_link_ori(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    
    X, Y, Z = quaternion_to_euler_angle(link_state.link_state.pose.orientation)
    link_ori_x = X
    link_ori_y = Y
    link_ori_z = Z
     
    return link_ori_x, link_ori_y, link_ori_z

def get_link_vel(link_name_main, reference_frame):
    global link_state
    
    link_state = get_link_state(link_name_main, reference_frame)
    link_vel_x = link_state.link_state.twist.angular.x
    link_vel_y = link_state.link_state.twist.angular.y
    link_vel_z = link_state.link_state.twist.angular.z
    
    return link_vel_x, link_vel_y, link_vel_z

# def gimbal_ori_vel():
    
#     global gimbal_ori_right_z
#     global gimbal_ori_left_z
#     global gimbal_vel_right_z
#     global gimbal_vel_left_z
#     global gimbal_ori
#     global gimbal_vel
#     global rate
    
#     if gimbal_ori_right_z > 0 and gimbal_ori_left_z < 0:
#         gimbal_ori = (abs(gimbal_ori_right_z)+abs(gimbal_ori_left_z))/2
#         gimbal_vel = (abs(gimbal_vel_right_z)+abs(gimbal_vel_left_z))/2
#         rate = rospy.Rate(100)
        
#     elif gimbal_ori_right_z < 0 and gimbal_ori_left_z > 0:
#         gimbal_ori = ((abs(gimbal_ori_right_z)+abs(gimbal_ori_left_z))/2) * (-1)
#         gimbal_vel = ((abs(gimbal_vel_right_z)+abs(gimbal_vel_left_z))/2) * (-1)
#         rate = rospy.Rate(100)
    
#     elif gimbal_ori_right_z < 0 and gimbal_ori_left_z < 0:
#         rate = rospy.Rate(10)
    
#     elif gimbal_ori_right_z > 0 and gimbal_ori_left_z > 0:
#         rate = rospy.Rate(10)
        
#     return gimbal_ori, gimbal_vel, rate

def theta_target(gimbal_deg):
    DEG2RAD = np.pi/180
    gimbal_deg = gimbal_deg*DEG2RAD
    
    theta_target = np.array([gimbal_deg])
    
    return theta_target

#################################################################################################    

imu_acc_data = [0,0,0]
imu_vel_data = [0,0,0]
imu_ori_data = [0,0,0]

###Right    
A_R = np.array([[0,0,1,0],
              [0,0,0,1],
              [71.67461173,0,0,-4.06516215],
              [0,0,0, 0]])
    
B_R = np.array([[0],[0],[0],[84.03361345]])

### Left
A_L = np.array([[0,0,1,0],
              [0,0,0,1],
              [71.67461173,0,0,4.06516215],
              [0,0,0, 0]])
    
B_L = np.array([[0],[0],[0],[-84.03361345]])

C = np.eye(4)

D = np.array([[0], [0], [0], [0]])
    
# q = [theta_R, theta_gb, theta_Rd, theta_gbd]
Q = sp.Matrix([ [2000,    0,    0,    0],
                [0,    1,    0,    0],
                [0,    0,    10,    0],
                [0,    0,    0,    1]])

R = sp.Matrix([ [0.45] ])

K_R, S_R, E_R = roll_K_gain_R()
ss0_R = [A_R, B_R, C, D]
sys0_R = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0_R])
sysc_R = sys0_R.feedback(K_R) 

K_L, S_L, E_L = roll_K_gain_L()
ss0_L = [A_L, B_L, C, D]
sys0_L = control.ss(*[pl.array(mat_i).astype(float) for mat_i in ss0_L])
sysc_L = sys0_L.feedback(K_L) 

RAD2DEG = 180/np.pi       
    
loop_cnt = 1

get_body = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

gimbal_state = []
Left_gimbal_name = 'Left_gimbal'
Right_gimbal_name = 'Right_gimbal'
Left_flywheel_name = 'Left_flywheel'
Right_flywheel_name = 'Right_flywheel'
gimbal_name_list = [Left_gimbal_name, Right_gimbal_name, Left_flywheel_name, Right_flywheel_name]

mid_name = 'hip_link'
low_name = 'ankle_roll_yaw_link'
top_name = 'cmg'
link_name_list = [mid_name, low_name, top_name]

 
body_state = []
body_name = 'wheeled_inverted_pendulum'
body_name_list = [body_name]

print('K: ', K_R)
#################################################################################################
    
if __name__ == '__main__':
    try:
        rospy.wait_for_service('gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
        rospy.init_node('CMG_inverted_pendulum', anonymous=True) 

        # pub_Lgb = rospy.Publisher('/wheeled_inverted_pendulum/left_gimbal/command', Float64, queue_size=100)
        pub_Rgb = rospy.Publisher('/wheeled_inverted_pendulum/right_gimbal/command', Float64, queue_size=100)
        pub_Lfw = rospy.Publisher('/wheeled_inverted_pendulum/left_flywheel/command', Float64, queue_size=100)
        pub_Rfw = rospy.Publisher('/wheeled_inverted_pendulum/right_flywheel/command', Float64, queue_size=100)
        pub_hip_roll = rospy.Publisher('/wheeled_inverted_pendulum/hip_roll/command', Float64, queue_size=100)
        pub_hip_pitch = rospy.Publisher('/wheeled_inverted_pendulum/hip_pitch/command', Float64, queue_size=100)
        pub_ankle_pitch = rospy.Publisher('/wheeled_inverted_pendulum/ankle_pitch/command', Float64, queue_size=100)
        pub_ankle_roll = rospy.Publisher('/wheeled_inverted_pendulum/ankle_roll/command', Float64, queue_size=100)
        # pub_ankle_yaw = rospy.Publisher('/wheeled_inverted_pendulum/ankle_yaw/command', Float64, queue_size=100)


        gazebo_setting()
        
        gimbal_ori = 0
        gimbal_vel = 0
        rate = rospy.Rate(100)
        
        pub_hip_roll.publish(0)
        pub_hip_pitch.publish(0)
        pub_ankle_pitch.publish(0)
        pub_ankle_roll.publish(0)
        # pub_ankle_yaw.publish(0)
        
        rpm = 5000
        flywheel_ang_vel = (rpm * 2 * np.pi)/60        
        flywheel_vel = np.array([flywheel_ang_vel])
        print(flywheel_vel)
        
        pub_Lfw.publish(flywheel_vel[0])
        pub_Rfw.publish(-flywheel_vel[0])
        
        ref = 0
        t = 0.01

        cur_time = time.time()             
        while not rospy.is_shutdown():     
            last_time = cur_time
            cur_time = time.time()
            dt = cur_time - last_time  
            
            gimbal_ori_left_x, gimbal_ori_left_y, gimbal_ori_left_z = get_link_ori(gimbal_name_list[0], link_name_list[2])
            gimbal_vel_left_x, gimbal_vel_left_y, gimbal_vel_left_z= get_link_vel(gimbal_name_list[0], link_name_list[2])
            gimbal_ori_right_x, gimbal_ori_right_y, gimbal_ori_right_z = get_link_ori(gimbal_name_list[1], link_name_list[2])
            gimbal_vel_right_x, gimbal_vel_right_y, gimbal_vel_right_z= get_link_vel(gimbal_name_list[1], link_name_list[2])
                        
            link_ori_x, link_ori_y, link_ori_z = get_link_ori(link_name_list[2], 'world')
            link_vel_x, link_vel_y, link_vel_z = get_link_vel(link_name_list[2], 'world')
            
            # # x0_L = np.array([link_ori_x,gimbal_ori_left_z,link_vel_x,gimbal_vel_left_z])
            x0_R = np.array([link_ori_x, gimbal_ori_right_z, link_vel_x, gimbal_vel_right_z])
            theta_R = theta_target(0)
            theta_L = theta_target(0)
            
            # if loop_cnt == 100:
            #     theta_R = theta_target(20)
                
            
            xd_R = np.array([0, theta_R[0], 0, 0])
            # xd_L = np.array([0, theta_L[0], 0, 0])
            
            u_R = - K_R @ (x0_R - xd_R) 
            # u_L = -K_L @ (x0_L - xd_L) 
            
            pub_Rgb.publish(u_R)
            # pub_Lgb.publish(-u_R)
                
            
            if loop_cnt % 10 == 0:
                flywheel_vel_x, flywheel_vel_y, flywheel_vel_z= get_link_vel(gimbal_name_list[2], gimbal_name_list[0])
                # gimbal_ori_x_l, gimbal_ori_y_l, gimbal_ori_z_l = get_link_vel(gimbal_name_list[0], link_name_list[2])
                
                print('Gimbal_angle_r      (deg): ', gimbal_ori_right_z*RAD2DEG)
                print('Gimbal_angle_l      (deg): ', gimbal_ori_left_z*RAD2DEG)
                print('FLywheel_velocity (rad/s): ', abs(flywheel_vel_y))
                print('Roll                (deg): ', link_ori_x*RAD2DEG)
                print('Yaw                 (deg): ', link_ori_z*RAD2DEG)
                
                # print(ang_acc)
                print('====================================')

            loop_cnt= loop_cnt + 1
                   
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
            # print('u_R: ', u_R)
            # print('u_L: ', u_L)
            # print('========================================')
            
            # if loop_cnt > 1000:
            #     pub_ankle.publish(0.05)
            #     pub_hip.publish(-0.05)
            # elif loop_cnt < 1000:
                # pub_ankle.publish(0.1)
                # pub_hip.publish(-0.1-link_ori_x)
            # elif loop_cnt < 1500:
            #     ref = -25
            # elif loop_cnt < 2000:
            #     ref = -30
            # elif loop_cnt < 2500:
            #     ref = -35
            # elif loop_cnt < 3000:
            #     ref = -40
            # print('u: ', u)
            # print('====================================')
                # print('Yaw                 (deg): ', body_ori_z*RAD2DEG)
            # if loop_cnt %200 ==0:
            #     ref = -200
            #     u = ref - K @ x0 
            #     pub_Lgb.publish(-u)
            #     pub_Rgb.publish(u)
                # print(loop_cnt)
                # print('====================================')
                # print('====================================')
