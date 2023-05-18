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
#%%
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
        
    

    #Check for point cloud
    #if 'pc2' in 
#%%
import time
#Now sort the files via their time:
sorted_is = np.argsort(times)
sorted_times = times[sorted_is]
sorted_files = subfiles[sorted_is]

#Now run through in order and display the point cloud as we move through it:
#Create figure
fig = plt.figure()
ax = plt.axes()
ax.set_xlabel("X")
ax.set_ylabel("Y")
for f in sorted_files:    
    if 'nparray' in f:
        with open(os.path.join(data_dir, f), 'rb') as handle:
            pointcloud = pickle.load(handle)
        xpts = pointcloud[0,:]
        ypts = pointcloud[1,:]
        zpts = pointcloud[2,:]
        
        ax.scatter(xpts, ypts, color='red')
        display(fig)
        clear_output(wait = True)
        plt.pause(.01)
        





# %%
