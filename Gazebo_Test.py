#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math 
import numpy as np
import scipy.io
import time

# Commanded velocity 
global l, num_of_iters,move
move = Twist() # defining the variable to hold values
move.linear.x = 0
move.linear.y = 0
move.linear.z = 0
move.angular.x = 0
move.angular.y = 0
move.angular.z = 0


l = 0.12
num_of_iters = 200

class Server:
    def __init__(self):
       self.iter = 0
       self.odom_iter = 0
       self.odom_x = None
       self.odom_y = None
       self.odom_zang = None
       self.odom_xin = None
       self.odom_yin = None
       self.odom_zangin = None
       
       self.t_mat = np.empty([1, num_of_iters])
       self.tt_mat = np.empty([1, num_of_iters])
       self.x_mat = np.empty([1, num_of_iters])
       #self.xx_mat = np.empty([num_of_iters,360])
       self.theta_mat = np.empty([1, num_of_iters])
       
       self.odom_t_mat = np.empty([1, num_of_iters*6])
       self.odom_tt_mat = np.empty([1, num_of_iters*6])
       self.odom_v_mat = np.empty([1, num_of_iters*6])
       self.odom_w_mat = np.empty([1, num_of_iters*6])
       self.odom_x_mat = np.empty([1, num_of_iters*6])
       self.odom_y_mat = np.empty([1, num_of_iters*6])
       self.odom_zang_mat = np.empty([1, num_of_iters*6])
    
    def lidar_callback(self, msg):
       t_k_sec = msg.header.stamp.secs
       t_k_nsec = msg.header.stamp.nsecs
       t_k = t_k_sec + t_k_nsec*10**(-9)
       x_array_k = msg.ranges
       
       x_array_0to44 = x_array_k[0:45]
       x_array_0to44_masked = np.ma.masked_equal(x_array_0to44, 0.0, copy=False)
       x_k_min1 = x_array_0to44_masked.min()
       theta_k_1  = x_array_0to44.index(x_k_min1)
       x_array_315to359 = x_array_k[315:360]
       x_array_315to359_masked = np.ma.masked_equal(x_array_315to359, 0.0, copy=False)
       x_k_min2 = x_array_315to359_masked.min()
       theta_k_2 = x_array_315to359.index(x_k_min2)
       
       if x_k_min1 <= x_k_min2:
           x_k = x_k_min1
           theta_k = theta_k_1
       else:
           x_k = x_k_min2
           theta_k = theta_k_2+315

       if self.iter < num_of_iters-1:
           self.t_mat[0,self.iter] = t_k
           self.tt_mat[0,self.iter] = time.time()
           self.x_mat[0,self.iter] = x_k
           #self.xx_mat[self.iter,:] = np.array(x_array_k)
           self.theta_mat[0,self.iter] =  theta_k
           
           xo = self.odom_x + l*np.cos(self.odom_zang)
           yo = self.odom_y + l*np.sin(self.odom_zang)
           
           print("lidar iter ", self.iter)
           self.controller(xo,yo)
           self.iter = self.iter + 1
       else:
           print("stop")
           move.linear.x = 0
           move.angular.z = 0
           
           scipy.io.savemat('Gazebo_Resp_bot2.mat', dict(t1=self.t_mat,tt1 = self.tt_mat, x1=self.x_mat, th1=self.theta_mat, O_t1=self.odom_t_mat,O_tt1=self.odom_tt_mat, O_v1=self.odom_v_mat, O_w1=self.odom_w_mat, O_x1=self.odom_x_mat,O_y1=self.odom_y_mat,O_zang1=self.odom_zang_mat))
     
    def odom_callback(self, msg):
       self.odom_vel = msg.twist.twist.linear.x
       self.odom_ang_vel = msg.twist.twist.angular.z
       self.odom_x = msg.pose.pose.position.x
       self.odom_y = msg.pose.pose.position.y
       orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
       euler = tf.transformations.euler_from_quaternion(orientation_list)
       self.odom_zang = euler[2]
       if self.odom_iter == 0:
           self.odom_xin = self.odom_x
           self.odom_yin = self.odom_y
           self.odom_zangin = self.odom_zang
       
       if self.odom_iter <= num_of_iters*6-1:
           o_tk = msg.header.stamp.secs
           o_tk_n = msg.header.stamp.nsecs
           self.odom_t_mat[0, self.odom_iter] = o_tk + o_tk_n*10**(-9)
           self.odom_tt_mat[0, self.odom_iter] = time.time()
           self.odom_v_mat[0, self.odom_iter] = self.odom_vel
           self.odom_w_mat[0, self.odom_iter] = self.odom_ang_vel
           self.odom_x_mat[0, self.odom_iter] = self.odom_x
           self.odom_y_mat[0, self.odom_iter] = self.odom_y
           self.odom_zang_mat[0, self.odom_iter] = self.odom_zang
           print(self.odom_iter , " x y zang ",[self.odom_x,self.odom_y,self.odom_zang]," xin yin zangin ",[self.odom_xin,self.odom_yin,self.odom_zangin])
           self.odom_iter = self.odom_iter + 1
               
    def controller(self,x,y):
       k1 = 0.1
       k2 = 0.1
       goalx = self.odom_xin + l*np.cos(self.odom_zangin)
       goaly = self.odom_yin + l*np.sin(self.odom_zangin) + 0.6
       xErr = goalx-x
       yErr = goaly-y
       dist = np.sqrt((goalx-x)**2 +(goaly-y)**2)
       print("dist ", dist)
       
       xdot = k1*xErr
       ydot = k2*yErr
       if np.absolute(dist) > 0.01:
          move.linear.x = xdot*np.cos(self.odom_zang)+ydot*np.sin(self.odom_zang)
          move.angular.z = (-xdot*np.sin(self.odom_zang)+ydot*np.cos(self.odom_zang))/(l)
       else:
          move.linear.x = 0
          move.angular.z = 0
#############################Edit only this function######
"""
def lidar_callback(msg): 
    global x,y,zang,l
    #Make sense of the callback data
    #rostopic echo msg
    #0 to 359 aray index values with each value at a degree increment 
    #range should be in meters
    #The readings start from left and go counter clockwise
    RANGES=msg.ranges
    #Get range measurements from the front left side of the robot [from 0degrees to 90degrees]
    left=list(RANGES[0:91])

    #Get range measurements from the front right side of the robot [from 270degrees to 359degrees]
    right=list(RANGES[270:])

    
    #Replace zero readings with some large number
    i=0
    while(i<len(left)):
        if(left[i]<=msg.range_min):
            left[i]=10000
        i+=1
        
    i=0
    while(i<len(right)):
        if(right[i]<=msg.range_min):
            right[i]=10000
        i+=1

    
    #Find the minimum range measurement from both sides
    min_range_left=min(left)
    min_range_right=min(right)

    xo = x + l*np.cos(zang)
    yo = y + l*np.sin(zang)
    controller(xo,yo,zang)
    
def callback(data):
    global x, y, zang,ite,xin,yin,zangin
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y 
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(orientation_list)
    zang = euler[2]
    
    if ite == 0:
        xin = x
        yin = y
        zangin = zang
    print(ite , " x y ",[x,y]," xin yin ",[xin,yin])
    ite = ite+1
    """
#############################Edit only this function######
  
rospy.init_node('Go_to_goal')  # Defines a node with name of Go_to_goal

server = Server()
velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_subscriber = rospy.Subscriber('/odom', Odometry, server.odom_callback)
lidar_subscriber = rospy.Subscriber('/scan', LaserScan, server.lidar_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown(): 
    velocity_pub.publish(move)
    rate.sleep()


