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
global l, num_of_iters,move1,move2,move3

l = 0.12
num_of_iters = 200

move1 = Twist() # defining the variable to hold values
move1.linear.x = 0
move1.linear.y = 0
move1.linear.z = 0
move1.angular.x = 0
move1.angular.y = 0
move1.angular.z = 0

move2 = Twist() # defining the variable to hold values
move2.linear.x = 0
move2.linear.y = 0
move2.linear.z = 0
move2.angular.x = 0
move2.angular.y = 0
move2.angular.z = 0

move3 = Twist() # defining the variable to hold values
move3.linear.x = 0
move3.linear.y = 0
move3.linear.z = 0
move3.angular.x = 0
move3.angular.y = 0
move3.angular.z = 0

class Server:
    def __init__(self,ID):
       self.ID = ID
       self.iter = 0
       self.odom_iter = 0
       self.odom_x = None
       self.odom_y = None
       self.odom_zang = None
       
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
           
           print("lidar iter ", self.iter)
           self.iter = self.iter + 1
       else:
           print("stop")
           
           if self.ID==2:
               scipy.io.savemat('Resp_bot2.mat', dict(ID = self.ID, t2=self.t_mat,tt2 = self.tt_mat, x2=self.x_mat, th2=self.theta_mat, O_t2=self.odom_t_mat,O_tt2=self.odom_tt_mat, O_v2=self.odom_v_mat, O_w2=self.odom_w_mat, O_x2=self.odom_x_mat,O_y2=self.odom_y_mat,O_zang2=self.odom_zang_mat))
           elif self.ID==3:
               scipy.io.savemat('Resp_bot3.mat', dict(ID = self.ID, t3=self.t_mat,tt3 = self.tt_mat, x3=self.x_mat, th3=self.theta_mat, O_t3=self.odom_t_mat,O_tt3=self.odom_tt_mat, O_v3=self.odom_v_mat, O_w3=self.odom_w_mat, O_x3=self.odom_x_mat,O_y3=self.odom_y_mat,O_zang3=self.odom_zang_mat))
           elif self.ID==4:
               scipy.io.savemat('Resp_bot4.mat', dict(ID = self.ID, t4=self.t_mat,tt4 = self.tt_mat, x4=self.x_mat, th4=self.theta_mat, O_t4=self.odom_t_mat,O_tt4=self.odom_tt_mat, O_v4=self.odom_v_mat, O_w4=self.odom_w_mat, O_x4=self.odom_x_mat,O_y4=self.odom_y_mat,O_zang4=self.odom_zang_mat))
        
    def odom_callback(self, msg):
        self.odom_vel = msg.twist.twist.linear.x
        self.odom_ang_vel = msg.twist.twist.angular.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation_list)
        self.odom_zang = euler[2]
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
          
    def RelController(self,poseid,RelPose,Form,goallist,Formerror,Disterror,move,lead,c_mat,ctime_mat,idd,ilist,PI_error,x_dotmat,x_err):
       dx = Form[0]
       dy = Form[1]
       l = 0.13
       goal_x = goallist[0]
       goal_y = goallist[1]
       distance_to_goal_x = goal_x-poseid[0]
       distance_to_goal_y = goal_y-poseid[1]
       
       distance_to_goal = np.sqrt(distance_to_goal_x**2+distance_to_goal_y**2)
       Disterror.append(distance_to_goal)
       #print("Dist_to_goal",distance_to_goal,"lead",lead)
       
       # Controller gain
       
       if lead==1:
          K_tr = 0.1
          K_tr_I = 0.0001
       else:
          K_tr = 0
          K_tr_I = 0
       K_for_p = 0.2
       K_for_I = 0
       K_ang = 0.1
       
       Relx = 0
       Rely = 0
       for i in ilist:
          Relx = Relx + RelPose[idd*5+i,0]
          Rely = Rely + RelPose[idd*5+i,1]
          
       x_form_error = dx - Relx
       y_form_error = dy - Rely
       
       Tot_distance_to_goal_x = PI_error[0]+distance_to_goal_x
       Tot_distance_to_goal_y = PI_error[1]+distance_to_goal_y
       Tot_x_form_error = PI_error[2]+x_form_error
       Tot_y_form_error = PI_error[3]+y_form_error
       #print(K_tr," ", lead)
       #print("x_error",x_form_error,"y_error",y_form_error,"PIx_error",Tot_x_form_error,"PIy_error",Tot_y_form_error)
       
       x_dot = K_for_p * (x_form_error) + K_tr*distance_to_goal_x + K_for_I * (Tot_x_form_error) + K_tr_I*Tot_distance_to_goal_x
       y_dot = K_for_p * (y_form_error) + K_tr*distance_to_goal_y + K_for_I * (Tot_y_form_error) + K_tr_I*Tot_distance_to_goal_y
       
       distance_to_pose = np.sqrt(x_form_error**2+y_form_error**2)
       Formerror.append(distance_to_pose)
       x_dotmat.append([x_dot,y_dot])
       x_err.append([x_form_error,y_form_error])
       #print("Dist_to_pose",distance_to_pose,"lead",lead)
       
       rob_theta = poseid[2]*180/np.pi
       if(rob_theta >179):
          rob_theta = rob_theta-360
       
       if lead==1:
          if distance_to_pose < 0.01 and distance_to_goal < 0.01:
             move.linear.x = 0
             move.angular.z = 0#K_ang*((rob_theta)*np.pi/180)
          else:
             move.linear.x = (x_dot*np.cos(rob_theta*np.pi/180) + y_dot*np.sin(rob_theta*np.pi/180))
             move.angular.z = -(-x_dot*np.sin(rob_theta*np.pi/180) + y_dot*np.cos(rob_theta*np.pi/180))/(l) 
       else:
          if distance_to_pose < 0.01:
             move.linear.x = 0
             move.angular.z = 0#K_ang*((rob_theta)*np.pi/180)
          else:
             move.linear.x = (x_dot*np.cos(rob_theta*np.pi/180) + y_dot*np.sin(rob_theta*np.pi/180))
             move.angular.z = -(-x_dot*np.sin(rob_theta*np.pi/180) + y_dot*np.cos(rob_theta*np.pi/180))/(l)
          
       ctime = time.time()
       c_mat.append([move.linear.x,move.angular.z])
       ctime_mat.append(ctime)
       
       PI_error[0]=Tot_distance_to_goal_x
       PI_error[1]=Tot_distance_to_goal_y
       PI_error[2]=Tot_x_form_error
       PI_error[3]=Tot_y_form_error
       
    def RelPos(self, poseid):
       r,c = poseid.shape
       RelPosition = np.empty([r*r,2])
       for i in range(r):
          for j in range(r):
             RelPosition[i*r+j,0] = poseid[i,0]-poseid[j,0]
             RelPosition[i*r+j,1] = poseid[i,1]-poseid[j,1]
       return RelPosition
       
    def Traj(self,t,start,v,tr):
       if tr==1:
          return list(start+v*t)
       elif tr==2:
          R = 1
          w = 0.01
          origin = start - np.array([R,0])
          pose = np.array([R*np.cos(w*t),R*np.sin(w*t)])
          return list(origin+pose)
       elif tr==0:
          off = np.array([0.6,0.6])
          return list(start+off)
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
if __name__ == '__main__':
    try:  
       rospy.init_node('Go_to_goal')  # Defines a node with name of Go_to_goal
       server1 = Server(2)
       server2 = Server(3)
       server3 = Server(4)
       
       velocity_pub1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       pose_subscriber1 = rospy.Subscriber('tb3_0/odom', Odometry, server1.odom_callback)
       lidar_subscriber1 = rospy.Subscriber('tb3_0/scan', LaserScan, server1.lidar_callback)
       velocity_pub2 = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=10)
       pose_subscriber2 = rospy.Subscriber('tb3_1/odom', Odometry, server2.odom_callback)
       lidar_subscriber2 = rospy.Subscriber('tb3_1/scan', LaserScan, server2.lidar_callback)
       velocity_pub3 = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=10)
       pose_subscriber3 = rospy.Subscriber('tb3_2/odom', Odometry, server3.odom_callback)
       lidar_subscriber3 = rospy.Subscriber('tb3_2/scan', LaserScan, server3.lidar_callback)
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
          velocity_pub1.publish(move1)
          velocity_pub2.publish(move2)
          velocity_pub3.publish(move3)
          rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass


