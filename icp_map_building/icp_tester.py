#%%
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
sorted_files = sorted_files[450:-1]
#Identify reference frame
ref_id = 2
with open(os.path.join(data_dir, sorted_files[ref_id]), 'rb') as handle:
    ref_pointcloud = pickle.load(handle)
ref_name = sorted_files[ref_id]
ref_2dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:]]).transpose()
ref_3dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:], ref_pointcloud[2,:]])


#ransac_line = pyrsc.Line()
#(A, B, inliers) = ransac_line.fit(ref_3dof.transpose())
clean_ref_3dof = icp.linesac(ref_3dof, include_radius=1)
fig = plt.figure()
ax = plt.axes()
#ax.scatter(ref_3dof[0,:], ref_3dof[1,:], color='red')
#ax.scatter(clean_ref_3dof[0,:], clean_ref_3dof[1,:], color='green')




sorted_files = sorted_files[2:-1]
pc_map = clean_ref_3dof[0:2, :].transpose()
Rmap_ref = np.identity(2)
tmap_obs = np.zeros((2,1))

fig = plt.figure()
ax =plt.axes()
ax.set_xlabel('X')
ax.set_ylabel('Y')
#ax.scatter(ref_2dof[:,0], ref_2dof[:,1], color='green')
ax.scatter(pc_map[:,0], pc_map[:,1], color='red')
ref_2dof = pc_map
#ax.scatter(pc_map[inliers, 0], pc_map[inliers,1], color='green')
c = 0
bOutlierRej = True
bDoubleSac = True

for f in sorted_files:    
    if 'nparray' in f and c % 1 == 0:
        print('Loading file: ', f)
        with open(os.path.join(data_dir, f), 'rb') as handle:
            obs_pointcloud = pickle.load(handle)
            
        
        obs_3dof = np.array([obs_pointcloud[0,:], obs_pointcloud[1,:], obs_pointcloud[2,:]])
        if bOutlierRej:
            obs_3dof = icp.linesac(obs_3dof, include_radius=1)
        #Size Nx2
        obs_2dof = np.array([obs_3dof[0,:], obs_3dof[1,:]]).transpose()

        #Run ICP to get pose info:
        #Expects size Nx2
        (Rref_obs, tref_obs, dont_use) = icp.run_icp(ref_2dof, obs_2dof, verbose=False)        
        #create pose BACK to original map frame
        if tref_obs[0] == 0:
            tmap_obs = tref_obs
        else:
            tmap_obs = tmap_obs + np.matmul(Rmap_ref, tref_obs)

        Rmap_obs = np.matmul(Rmap_ref, Rref_obs)
        #Transform back to map frame
        tmp_pc_map = np.matmul(Rmap_obs, obs_2dof.transpose()) + tmap_obs
        #Re-run to original map:
        if bDoubleSac:
            (Rblah, tblah, tmp_pc_map) = icp.run_icp(pc_map, tmp_pc_map.transpose(), verbose=False)

        #Prepare for next frame
        ref_2dof = obs_2dof
        Rmap_ref = Rmap_obs
        pc_map = np.vstack((pc_map, tmp_pc_map))
        ax.scatter(tmp_pc_map[:,0], tmp_pc_map[:,1], color='blue')
        if c > 100:
            break
        print('c: ', c)
    c+=1

# %%
