#%%
#import open3d as o3d
import pyransac3d as pyrsc
import pickle
import sys
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from IPython.display import display, clear_output

#from deepracer_interfaces_pkg.msg import CameraMsg
#from cv_bridge import CvBridge
import cv2
import os
import re
import numpy as np
import datetime
import icp
import math
#%%
data_dir = '../navigation_module/navigator_host'
# %%
#Some basic debugging
with open(os.path.join(data_dir, 'map_file.pickle'), 'rb') as handle:
    pts = pickle.load(handle)
with open(os.path.join(data_dir, 'debug_map.pickle'), 'rb') as handle:
    debug_points = pickle.load(handle)


'''
Position: <2.3807 -3.3426 0.0000>
[navigator_host-1] [INFO] [1683248388.636508109] [navigator_host]: Quaternion: <0.0000 0.0000 0.7314, 0.6819>
'''


# %%
pos = np.array([[3.5733, -2.9475]]).transpose()
rot = [0.0000, 0.0000, 0.0837, 0.9965]
goal = np.array([[5, 0]]).transpose()


#hand_path = np.array([[5, 0], [5, -1], [5, -2], [4.5, -2.5]]).transpose()
hand_path = np.array([[4.5, -2.5], [5, -2], [5, -1], [5, 0]]).transpose()
hand_path = np.hstack((pos, hand_path))
hand_path = np.hstack((hand_path, goal))
#print(hand_path)

sample_num = 5
total_pts = np.array([])
for i in range(0, hand_path.shape[1]-1):
    st_x = hand_path[0, i]
    st_y = hand_path[1, i]
    ed_x = hand_path[0, i + 1]
    ed_y = hand_path[1, i + 1]
    pts_x = np.linspace(st_x, ed_x, num=sample_num)
    pts_y = np.linspace(st_y, ed_y, num=sample_num)
    if np.any(total_pts):
        total_pts = np.hstack((total_pts, np.array([pts_x[1:-1], pts_y[1:-1]])))
    else:
        total_pts = np.array([pts_x[1:-1], pts_y[1:-1]])

#%%
print(icp.euler_from_quaternion(rot[0], rot[1], rot[2], rot[3]))

plt.plot(pts[:,0],pts[:,1], '*', color='red', markersize=5)
plt.plot(pos[0], pos[1], 'x', color='green', markersize=5)
plt.plot(total_pts[0,:], total_pts[1,:], '.', color='green')
plt.plot(debug_points[0,:], debug_points[1,:], '.', color='blue', markersize=5)
plt.axis('equal')
plt.show()

# %%
