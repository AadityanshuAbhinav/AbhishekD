#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import scipy.io
import cv2.aruco as aruco
import time
import scipy.io
import tf
from geometry_msgs.msg import Twist

global s,ite,move1,poseid,poseidin,flaglist,initial_time
s = 0.52
id_list = None
Coordinates_list = None
z_anglist = None
poseid = np.zeros([5,3])
poseidin = np.zeros([5,3])

num_of_iters = 150
id_mat = []
Coordinates_mat = []
z_ang_mat = []
poseid_mat = []
time_mat = []
ctime_mat = []
c_mat = []
dist_mat = []
dist_facmat = []
Oe_mat = []
ite = 0

flaglist = [0,0,0,0,0]

# Commanded velocity 
move1 = Twist() # defining the variable to hold values
move1.linear.x = 0
move1.linear.y = 0
move1.linear.z = 0
move1.angular.x = 0
move1.angular.y = 0
move1.angular.z = 0
    

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
           
           #print("lidar iter ", self.iter)
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
    
class IntelSubscriber:
    def __init__(self):       
       self.bridge = CvBridge()
       self.rgb_sub = rospy.Subscriber('/realsense/camera/rgb/image_raw', Image, self.IntelSubscriberRGB)
    
    def IntelSubscriberRGB(self,msg):
       global ite,flaglist,move1,ctime_mat,c_mat,dist_mat,poseidin,time_mat,poseid_mat,Oe_mat,initial_time,dist_facmat
       t_k = time.time()
       
       if ite == 0:
          initial_time = t_k
          
       time_mat.append(t_k-initial_time)
         
       try:
          rospy.loginfo(rospy.get_caller_id() + "Recieving RGB data")
          rgb_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
          cv2.waitKey(1)
       except Exception as e:
          print(e)
          
       camera_msg = rospy.wait_for_message('/realsense/camera/rgb/camera_info', CameraInfo)
       depth_msg = rospy.wait_for_message('/realsense/camera/depth/image_raw', Image)
       depth_image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='passthrough')
       
       camera_matrix = np.array(camera_msg.K).reshape((3,3))
       dist_coeff = np.array(camera_msg.D)
       intrinsics = rs.intrinsics()
       intrinsics.width = 640
       intrinsics.height = 480
       intrinsics.fx = camera_matrix[0,0]
       intrinsics.fy = camera_matrix[1,1]
       intrinsics.ppx = camera_matrix[0,2]
       intrinsics.ppy = camera_matrix[1,2]
       
       aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
       aruco_params = cv2.aruco.DetectorParameters()
       aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
       
       corners, ids, _ = aruco_detector.detectMarkers(rgb_image)
       if ids is not None:
          for i in range(len(ids)):
             if ids[i,0] == 1:
                rvec, tvec,z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                p1 = center
                depth_value = depth_image[center[1], center[0]]
                
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   x1_ = depth_point[0]
                   y1_ = depth_point[1]
                   z1 = depth_point[2] # no need for separate z1_, as its value won't be changed
                   org_a = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
             
             elif ids[i,0] == 2:
                rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, None)
                center = np.mean(corners[i][0], axis=0).astype(int)
                depth_value = depth_image[center[1], center[0]]
                p2 = center
                
                if depth_value > 0:
                   depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)
                   x2_ = depth_point[0]
                   y2_ = depth_point[1]
                   z2 = depth_point[2]
                   org_b = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)
          x1_, y1_, x2_,y2_ = round(x1_,2),round(y1_,2), round(x2_,2), round(y2_,2)
          calib_coords = [x1_, y1_, x2_,y2_]
          theta = self.angle(y1_,y2_,x1_,x2_)

          if (1 in ids) and (2 in ids):
              x1,y1 = self.coordinates(calib_coords,x1_,y1_,theta)
              x2,y2 = self.coordinates(calib_coords,x2_,y2_,theta)
              marker_text_a = "A({:.2f}, {:.2f})".format( x1, y1)
              marker_text_b = "B({:.2f}, {:.2f})".format(x2, y2)
              cv2.line(rgb_image, p1, p2,(255,0,0), 1)
          else:
              print("Reference tags are not in view")
              
       id_list = []
       Coordinates_list = []
       z_anglist = []
       poseid = np.empty([5,3])                 
       if ids is not None:
          for i in range(len(ids)):
             rvec, tvec, z_ang, _ = self.my_estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix,None)
             #print(i," ",tvec)
             center = np.mean(corners[i][0], axis=0).astype(int)
             depth_value = depth_image[center[1], center[0]]
             
             if depth_value > 0:
                depth_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center[0], center[1]], depth_value)

                marker_text = "Marker ID: {} | Coordinates: {:.2f}, {:.2f}, {:.2f}".format(ids[i][0], depth_point[0], depth_point[1], depth_point[2])
                   
                marker_x_ = depth_point[0]
                marker_y_ = depth_point[1]
                marker_z = depth_point[2]
                  
                #using the calibrated values
                marker_x,marker_y = self.coordinates(calib_coords,marker_x_,marker_y_,theta)
                
                z_ang = z_ang-np.pi/2 
                  
                if z_ang<0:
                   z_ang = z_ang+2*np.pi
                id_list.append(ids[i,0])
                Coordinates_list.append([marker_x,marker_y])
                z_anglist.append(z_ang*180/np.pi)
                # Robot ids
                
                if flaglist[ids[i,0]-1] == 0:
                   poseidin[ids[i,0]-1,0] = marker_x
                   poseidin[ids[i,0]-1,1] = marker_y
                   poseidin[ids[i,0]-1,2] = z_ang
                   flaglist[ids[i,0]-1] = 1
                elif flaglist[ids[i,0]-1] == 1:
                   poseid[ids[i,0]-1,0] = marker_x
                   poseid[ids[i,0]-1,1] = marker_y
                   poseid[ids[i,0]-1,2] = z_ang
                
                marker_text = "ID: {} ({:.2f}, {:.2f})".format(ids[i][0], marker_x, marker_y)
                # Convert coordinates to integers before passing to putText
                org = (int(corners[i][0, 0, 0]), int(corners[i][0, 0, 1]) - 10)

                cv2.putText(rgb_image, marker_text, org,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                cv2.aruco.drawDetectedMarkers(rgb_image, corners)  
       cv2.imshow("ArUco Marker Detection", rgb_image)
       print("id list",id_list,"Coordinates list",Coordinates_list)
       # Exit the program when the 'Esc' key is pressed
       #if cv2.waitKey(1) & 0xFF == 27:
          #break
       id_mat.append(id_list)
       Coordinates_mat.append(Coordinates_list)
       z_ang_mat.append(z_anglist)
       poseid_mat.append(list(poseid))
       ite = ite+1
       
       #self.controller(poseid[4,0],poseid[4,1],poseid[4,2],poseidin[4,0],poseidin[4,1],poseidin[4,2],ctime_mat,c_mat,dist_mat,Oe_mat,move1)       
       
       current_time = time.time()
       if (current_time > initial_time + 4 ):
          id_mat1 = np.array(id_mat)
          Coordinates_mat1 = np.array(Coordinates_mat)
          z_ang_mat1 = np.array(z_ang_mat)
          time_mat1 = np.array(time_mat)
          c_mat11 = np.array(c_mat)
          ctime_mat11 = np.array(ctime_mat)
          dist_mat11 = np.array(dist_mat)
          poseid_mat11 = np.array(poseid_mat)
          Oe_mat11 = np.array(Oe_mat)
          scipy.io.savemat('Aruco.mat', dict(idmat=id_mat1, Coodmat=Coordinates_mat1, zmat=z_ang_mat1,cmat1=c_mat11, ctimemat1 = ctime_mat11, timemat = time_mat1,distmat1 = dist_mat11,poseidmat = poseid_mat11,oemat = Oe_mat11))

    def coordinates(self, calib_coords,x_,y_,theta):
       x1,y1,x2,y2 = calib_coords
       tan = np.tan(theta)
       cos = np.cos(theta)
       sec = 1/(np.cos(theta))
       cosec = 1/(np.sin(theta))
       y_tmp = (y_ - y1 - (x_- x1)*tan)*sec
       x_tmp = (x_ - x1)*sec + y_tmp*tan
       s_ = np.sqrt((y2-y1)**2 + (x2-x1)**2)
       dist_fac = 1000
       x = x_tmp/dist_fac
       y = y_tmp/dist_fac
       return x,y
    
    def angle(self, y1_,y2_,x1_,x2_):
       invtan = np.arctan2(float(y2_-y1_),float(x2_-x1_))
       return invtan
    
    def my_estimatePoseSingleMarkers(self, corners, marker_size, mtx, distortion):
       marker_points = np.array([[-marker_size / 2, marker_size / 2, 0], [marker_size / 2, marker_size / 2, 0], [marker_size / 2, -marker_size / 2, 0], [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
       trash = []
       rvecs = []
       tvecs = []
       z_angle = 0
       for c in corners:
          nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
          RR,_ = cv2.Rodrigues(R)
          sy = np.sqrt(RR[0,0]*RR[0,0]+RR[1,0]*RR[1,0])
          if sy>1e-6:
              z_angle = z_angle+np.arctan2(RR[1,0],RR[0,0])
          else:
              z_angle = z_angle
          
          rvecs.append(R)
          tvecs.append(t)
          trash.append(nada)
       #z_angle = z_angle/4
       return rvecs, tvecs, z_angle, trash
    
    def controller(self,x,y,zang,xin,yin,zangin,ctime_mat,c_mat,dist_mat,Oe_mat,move):
       global initial_time
       alpha = 0.1
       k = 0.1
       goalx = xin+0.6
       goaly = yin 
       dist = np.sqrt((goalx-x)**2 +(goaly-y)**2)
       desired_orientation = np.arctan2((goaly - y),(goalx - x))
       orientation_error = desired_orientation - zang
       orientation_error%=(2*np.pi)
       if orientation_error>np.pi:
          orientation_error-= 2*np.pi
          
       if np.absolute(dist) > 0.01:
          move.linear.x = alpha*dist
          move.angular.z = -k*orientation_error
       else:
          move.linear.x = 0
          move.angular.z = 0
       ctime = time.time()-initial_time
       dist_mat.append(dist)
       Oe_mat.append(orientation_error)
       c_mat.append([move.linear.x,move.angular.z])
       ctime_mat.append(ctime)
       
    def Traj(self,t,start,v,tr):
       if tr==1:
          return list(start+v*t)
       elif tr==2:
          origin = start - np.array([0.5,0])
          pose = np.array([0.5*cos(0.1*t),0.5*sin(0.1*t)])
          return list(origin+pose)

if __name__ == '__main__':
    try:
       rospy.init_node('realsense_subscriber_rgb',anonymous=True)
       rs_subscriber = IntelSubscriber()
       
       velocity_pub1 = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=10)
       rate = rospy.Rate(10)
       while not rospy.is_shutdown():
          velocity_pub1.publish(move1)
          rate.sleep()
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
