#%%
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

# %%
YEAR = 2022
MONTH = 3

data_dir = '/home/taylor/SJSU-ANS_DeepRacer/data/long_little_better'
files = os.listdir(data_dir)
times = np.array([])
subfiles = np.array([])
for f in files:
    #split
    file_time = re.findall('\d\d', f)
    if file_time is not None:
        day = int(file_time[0])
        hour = int(file_time[1])
        minute = int(file_time[2])
        second = int(file_time[3])
        ms = int(file_time[4] + file_time[5])
        #create datetime
        file_time_dt = datetime.datetime(YEAR, MONTH,day, hour , minute, second, ms)
        times = np.append(times, file_time_dt)
        subfiles = np.append(subfiles, f)

sorted_is = np.argsort(times)
sorted_times = times[sorted_is]
sorted_files = subfiles[sorted_is]
#%%
#First do a simple test, the first and second point clouds...
with open(os.path.join(data_dir, sorted_files[1]), 'rb') as handle:
    ref_pointcloud = pickle.load(handle)
ref_2dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:]]).transpose()


#ICP for the win
fig = plt.figure()
ax = plt.axes()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.scatter(ref_2dof[:, 0], ref_2dof[:,1], color='red')


with open(os.path.join(data_dir, sorted_files[77]), 'rb') as handle:
            obs_pointcloud = pickle.load(handle)
obs_2dof = np.array([obs_pointcloud[0,:], obs_pointcloud[1,:]]).transpose()
(history, aligned_pts, R, t) = icp.run_icp(ref_2dof, obs_2dof, verbose=True)
ax.scatter(aligned_pts[:,0], aligned_pts[:,1], color='green')
ax.scatter(obs_2dof[:,0], obs_2dof[:,1], color='blue')
'''
To use ICP in a series of points need to align each point cloud successively, to not
have to do this a lot we need to chain the R,t togheter. The following for loop was making sure
I could do this. The rotation seems correct, but seomthing is off int erms of the translation
So come and figure that out!
'''
'''
t = np.zeros((2,1))
R = np.identity(2)
for h in history:
    R1_0 = h[0:2,0:2]
    t1_0 = h[0:2, 2].reshape(2,1)
    #print(t1_0)
    #t = t1_0+np.matmul(R, t)
    R = np.matmul(R1_0 , R)
    if t[0] == 0:
        t = t1_0
    else:
        t = t1_0 + np.matmul(R1_0, t)

    #t = t+t1_0 #np.matmul(R.transpose(), t1_0)
    #print('Summ')
    #print(t)
'''
#Make sure we understand translation/rotation
new_aligned_pts = np.matmul(R, (obs_2dof.transpose())) + t
#new_aligned_pts = obs_2dof.transpose()
ax.scatter(new_aligned_pts[0,:], new_aligned_pts[1,:], color='black')
plt.show()


#iteration 1
#R = R1_0
#t goes from 0 to 1
#iteration 2
#t goes from 1 to 2
#R2_0 = R2_1 * R1_0

#%%

for f in sorted_files:
    if 'nparray' in f:
        with open(os.path.join(data_dir, sorted_files[77]), 'rb') as handle:
            obs_pointcloud = pickle.load(handle)

        obs_2dof = np.array([obs_pointcloud[0,:], obs_pointcloud[1,:]]).transpose()
        (history, aligned_pts) = icp.run_icp(ref_2dof, obs_2dof, verbose=False)

#ICP for the win
fig = plt.figure()
ax = plt.axes()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.scatter(ref_2dof[:, 0], ref_2dof[:,1], color='red')
ax.scatter(obs_2dof[:,0], obs_2dof[:,1], color='blue')






#ax.scatter(ref_2dof[:, 0], ref_2dof[:,1], color='red')
#ax.scatter(obs_2dof[:,0], obs_2dof[:,1], color='blue')
ax.scatter(aligned_pts[:,0], aligned_pts[:,1], color='green')
plt.show()
display(fig)

# %%
