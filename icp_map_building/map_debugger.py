#%%
#import open3d as o3d


import pyransac3d as pyrsc
import pickle
import sys
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import rclpy 
from datetime import datetime
from scipy.spatial.transform import Rotation as R


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_map_server/map_io.hpp>
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Header

#from deepracer_interfaces_pkg.msg import CameraMsg
#from cv_bridge import CvBridge
import cv2
import os
import re
import numpy as np
import icp
import math
#%%
data_dir = 'navigation_module/navigator_host'
# %%
#Some basic debugging
with open(os.path.join(data_dir, 'map_file.pickle'), 'rb') as handle:
    pts = pickle.load(handle)
with open(os.path.join(data_dir, 'debug_map.pickle'), 'rb') as handle:
    debug_points = pickle.load(handle)


# %%
pos = np.array([[3.5733, -2.9475]]).transpose()
rot = [0.0000, 0.0000, 0.0837, 0.9965]
goal = np.array([[5, 0]]).transpose()


#hand_path = np.array([[5, 0], [5, -1], [5, -2], [4.5, -2.5]]).transpose()
hand_path = np.array([[4.5, -2.5], [5, -2], [5, -1]]).transpose()
hand_path = np.hstack((pos, hand_path))
hand_path = np.hstack((hand_path, goal))
#print(hand_path)

sample_num = 5
total_pts = np.array([])
total_rots = -1
for i in range(0, hand_path.shape[1]-1):
    st_x = hand_path[0, i]
    st_y = hand_path[1, i]
    ed_x = hand_path[0, i + 1]
    ed_y = hand_path[1, i + 1]

    #define this vector
    vect = np.array([ed_x - st_x,ed_y - st_y])
    x_unit = np.array([1, 0])
    rotangle = np.arccos(np.dot(vect, x_unit)/(np.linalg.norm(x_unit) * np.linalg.norm(vect)))
    
    pts_x = np.linspace(st_x, ed_x, num=sample_num)
    pts_y = np.linspace(st_y, ed_y, num=sample_num)
    if np.any(total_pts):
        total_pts = np.hstack((total_pts, np.array([pts_x[0:-1], pts_y[0:-1]])))
        total_rots = np.hstack((total_rots, np.ones(sample_num)*rotangle))
        
    else:
        total_pts = np.array([pts_x[1:-1], pts_y[1:-1]])
        total_rots = np.ones(sample_num) * rotangle

total_rots = np.array(total_rots)
#%%
print(icp.euler_from_quaternion(rot[0], rot[1], rot[2], rot[3]))

plt.plot(pts[:,0],pts[:,1], '*', color='red', markersize=5)
plt.plot(pos[0], pos[1], 'x', color='green', markersize=5)
plt.plot(total_pts[0,:], total_pts[1,:], '.', color='green')
plt.plot(debug_points[0,:], debug_points[1,:], '.', color='blue', markersize=5)
plt.axis('equal')
plt.show()

#Now convert to the appropriate parameters
epoch = datetime.now()
myHeader = Header()
myHeader.stamp.sec = 0
myHeader.stamp.nanosec = 0
myHeader.frame_id = "odom"
myPath = Path()
myPath.header = myHeader
for i in range(0, total_pts.shape[1]):
    pt = total_pts[:, i]
    yaw = total_rots[i]
    Q = Quaternion()
    tmpq = icp.get_quaternion_from_euler(0, 0, yaw)
    Q.x = tmpq[0]
    Q.y = tmpq[1]
    Q.z = tmpq[2]
    Q.w = tmpq[3]

    P = Point()
    P.x = total_pts[0,i]
    P.y = total_pts[1,i]
    tmpPose = Pose()
    tmpPose.position = P
    tmpPose.orientation = Q
    myHeader = Header()
    myHeader.stamp.sec = int(np.floor((datetime.now() - epoch).total_seconds()))
    #Using miliseconds as nanoseconds
    myHeader.stamp.nanosec = int(np.floor((datetime.now() - epoch).total_seconds() * 10**6))
    myHeader.frame_id = "odom"
    tmpPS = PoseStamped()
    tmpPS.header = myHeader
    tmpPS.pose = tmpPose
    myPath.poses.append(tmpPS)


first_pose = myPath.poses[0]
q = first_pose.pose.orientation
p = first_pose.pose.position
r = R.from_quat([q.x, q.y, q.z, q.w])
rmat = r.as_matrix()
np.matmul(rmat, np.array([1, 0, 0]))

