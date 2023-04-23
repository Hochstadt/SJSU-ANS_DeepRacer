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
import math

YEAR = 2022
MONTH = 3

data_dir = '/home/taylor/SJSU-ANS_DeepRacer/data/slow_steady_clunky'
files = os.listdir(data_dir)
times = np.array([])
subfiles = np.array([])
for f in files:
    #split
    file_time = re.findall('\d\d', f)
    if file_time is not None and len(file_time) > 2:
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
#Shorten because need to skip intor
sorted_files = sorted_files[70:-1]
#Identify reference frame
start_index = 300
for f in sorted_files:
    if 'nparray' in f:
        ref_id = start_index
        break
    start_index+=1




with open(os.path.join(data_dir, sorted_files[ref_id]), 'rb') as handle:
    ref_pointcloud = pickle.load(handle)
ref_name = sorted_files[ref_id]
ref_2dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:]]).transpose()
ref_3dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:], ref_pointcloud[2,:]])


#ransac_line = pyrsc.Line()
#(A, B, inliers) = ransac_line.fit(ref_3dof.transpose())
clean_ref_3dof = icp.linesac(ref_3dof, include_radius=1)
sorted_times = sorted_times[ref_id + 1:-1]

sorted_files = sorted_files[ref_id + 1:-1]
pc_map = clean_ref_3dof[0:2, :].transpose()
full_pc_map = []
Rest_obs = np.identity(2)
test_obs = np.zeros((2,1))
translation = []
rotation_angle  = []
sensor_loc = []
outlier_metric = []
time = []
#fig = plt.figure()
#ax =plt.axes()
#ax.set_xlabel('X')
#ax.set_ylabel('Y')
c = 0
bOutlierRej = False
bDoubleSac = True
IDLE_TIME = 30

for f in sorted_files:    
    if 'nparray' in f and c % 1 == 0:
        time.append(sorted_times[c])
        with open(os.path.join(data_dir, f), 'rb') as handle:
            obs_pointcloud = pickle.load(handle)
            
        
        obs_3dof = np.array([obs_pointcloud[0,:], obs_pointcloud[1,:], obs_pointcloud[2,:]])
        if bOutlierRej:
            obs_3dof = icp.linesac(obs_3dof, include_radius=1)
            
        #Size Nx2
        obs_2dof = np.array([obs_3dof[0,:], obs_3dof[1,:]]).transpose()
        #obs_2dof = icp.trimPointsV2(obs_2dof, scan_radius = .05, POINT_THRESH=1)

        #Run ICP to get pose info:
        #Expects size Nx2
        #Attempt transform backt o map frame:
        est_pc_map = np.matmul(Rest_obs, obs_2dof.transpose()) + test_obs                    
        (Rmap_est, tmap_est, dont_use, outlier_mask) = icp.run_icp(pc_map, est_pc_map.transpose(), .1, 
                                                                   distance_threshold=.5, verbose=False)        

        #Print the rotation angle and translation        
        rot_angle = math.acos(Rmap_est[0, 0])
        rot_angle_check = math.asin(Rmap_est[1,0])
        print('Iteration: ', c, 'file: ', f)
        print('Rotation is: ', math.degrees(rot_angle))        
        print('Translation is: <', tmap_est[0], ',', tmap_est[1], '>')
        #Add sanity check, shouldn't be moving all that fast (10deg )
        #create pose BACK to original map frame, larger than 8deg issue
        #larger than .3m issue
        delta_t = math.sqrt(tmap_est[0]**2 + tmap_est[1]**2)
    

        tmap_obs = tmap_est + np.matmul(Rmap_est, test_obs)
        Rmap_obs = np.matmul(Rmap_est, Rest_obs)

        #Transform back to map frame
        tmp_pc_map = np.matmul(Rmap_obs, obs_2dof.transpose()) + tmap_obs        
        #Transpose back to do a vstack
        tmp_pc_map = tmp_pc_map.transpose()        
        sensor_origin =np.matmul(Rmap_obs, np.zeros((2,1))) + tmap_obs
        
        #Data saving
        sensor_loc.append(sensor_origin)
        rotation_angle.append(math.degrees(rot_angle))
        translation.append(tmap_est)

        #Prepare for next frame        
        Rest_obs = Rmap_obs
        test_obs = tmap_obs
        #Only add to the map if the outlier count is below 10        
        pc_map = np.vstack((pc_map, tmp_pc_map[outlier_mask, :]))

        if c > 1200:
            break
        #plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
        #plt.scatter(tmp_pc_map[np.invert(outlier_mask), 0], tmp_pc_map[np.invert(outlier_mask), 1], color='red')
        #plt.show()

    #if c > 20 and c % 10 == 0:
    if c % 100 == 0 and c > 1:
    #    plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
        pc_map = icp.trimPointsV2(pc_map, scan_radius = .1, POINT_THRESH=25)
        #Add the pc map to the full one:
        if full_pc_map == []:
            full_pc_map = np.copy(pc_map)
        else:
            full_pc_map = np.vstack((full_pc_map, pc_map))
        pc_map = np.copy(full_pc_map)

    #    plt.scatter(pc_map[:,0], pc_map[:,1], color='green')
    #    plt.show()

        #Trim points out of map
        #pc_map = icp.trimPoints(pc_map, include_radius=.05)
    if c % 20 == 0 and c > 1:
        plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
        if not full_pc_map == []:
            plt.scatter(full_pc_map[:,0], full_pc_map[:,1], color='green')
        tmp_sensor = np.array(sensor_loc)
        tmp_sensor = tmp_sensor.reshape((tmp_sensor.shape[0], tmp_sensor.shape[1]))
        plt.scatter(tmp_sensor[:,0], tmp_sensor[:,1], color='magenta')
        plt.show()
        

    c+=1

plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
#plt.scatter(tmp_pc_map[np.invert(outlier_mask), 0], tmp_pc_map[np.invert(outlier_mask), 1], color='red')

#redux_pc_map = icp.trimPointsV2(pc_map, scan_radius = .1, POINT_THRESH=20)
#plt.scatter(redux_pc_map[:,0], redux_pc_map[:,1], color='green')
plt.show()
# %%
#Reepat
pc_map = redux_pc_map
#c = 0
shorted_sorted_files = sorted_files[c:-1]
#Rest_obs = np.identity(2)
#test_obs = np.zeros((2,1))
#%%
for f in shorted_sorted_files:    
    if 'nparray' in f and c % 1 == 0:
        time.append(sorted_times[c])
        with open(os.path.join(data_dir, f), 'rb') as handle:
            obs_pointcloud = pickle.load(handle)
            
        
        obs_3dof = np.array([obs_pointcloud[0,:], obs_pointcloud[1,:], obs_pointcloud[2,:]])
        if bOutlierRej:
            obs_3dof = icp.linesac(obs_3dof, include_radius=1)
            
        #Size Nx2
        obs_2dof = np.array([obs_3dof[0,:], obs_3dof[1,:]]).transpose()

        #Run ICP to get pose info:
        #Expects size Nx2
        #Attempt transform backt o map frame:
        est_pc_map = np.matmul(Rest_obs, obs_2dof.transpose()) + test_obs                    
        
        (Rmap_est, tmap_est, dont_use, outlier_mask) = icp.run_icp(pc_map, est_pc_map.transpose(), .1, 
                                                                   distance_threshold=.5, verbose=False)        
        #Print the rotation angle and translation        
        rot_angle = math.acos(Rmap_est[0, 0])
        rot_angle_check = math.asin(Rmap_est[1,0])
        print('Iteration: ', c, 'file: ', f)
        print('Rotation is: ', math.degrees(rot_angle))        
        print('Translation is: <', tmap_est[0], ',', tmap_est[1], '>')

        #create pose BACK to original map frame

        tmap_obs = tmap_est + np.matmul(Rmap_est, test_obs)
        Rmap_obs = np.matmul(Rmap_est, Rest_obs)

        #Transform back to map frame
        tmp_pc_map = np.matmul(Rmap_obs, obs_2dof.transpose()) + tmap_obs        
        #Transpose back to do a vstack
        tmp_pc_map = tmp_pc_map.transpose()        
        sensor_origin =np.matmul(Rmap_obs, np.zeros((2,1))) + tmap_obs
        
        #Data saving
        sensor_loc.append(sensor_origin)
        rotation_angle.append(math.degrees(rot_angle))
        translation.append(tmap_est)

        #Prepare for next frame        
        Rest_obs = Rmap_obs
        test_obs = tmap_obs
        #Only add to the map if the outlier count is below 10        
        pc_map = np.vstack((pc_map, tmp_pc_map[outlier_mask, :]))

        if c > 1100:
            break

    if c % 20 == 0 and c > 1:
        plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
        tmp_sensor = np.array(sensor_loc)
        tmp_sensor = tmp_sensor.reshape((tmp_sensor.shape[0], tmp_sensor.shape[1]))
        plt.scatter(tmp_sensor[:,0], tmp_sensor[:,1], color='magenta')
        plt.show()
        

    c+=1

plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
#plt.scatter(tmp_pc_map[np.invert(outlier_mask), 0], tmp_pc_map[np.invert(outlier_mask), 1], color='red')

redux_pc_map = icp.trimPointsV2(pc_map, scan_radius = .1, POINT_THRESH=10)
plt.scatter(redux_pc_map[:,0], redux_pc_map[:,1], color='green')
plt.show()


# %%
#Post process point cloud to select most frequenyt and thus most likely measuremnts
max_x = max(pc_map[:,0])
max_y = max(pc_map[:,1])
min_x = min(pc_map[:,0])
min_y = min(pc_map[:,1])
#Create a grid for ever Xm across the map
spacing = .1
#Unsure how this will go, but need to come back ad figure out how to reduce the map
#Right now, before leaving the pc_map is in memory alreadyloaded




# %%
#Try ot ransac again
newpc_map = np.append(pc_map, np.zeros((pc_map.shape[0], 1)), axis=1)
pc_map_refined = icp.linesac(newpc_map.transpose(), include_radius = 1)
pc_map_refined = pc_map_refined.transpose()
#%%
fig = plt.figure()
ax =plt.axes()
ax.scatter(pc_map[:,0], pc_map[:,1], color='red')
ax.scatter(pc_map_refined[:,0], pc_map_refined[:,1], color='green')
#%%

rotation_angle = np.array(rotation_angle)
plt.plot(rotation_angle)
plt.show()
#%%
translation = np.array(translation)
translation = translation.reshape((translation.shape[0], translation.shape[1]))
plt.plot(translation[:,0], color='blue')
plt.plot(translation[:,1], color='green')

# %%
outlier_metric = np.array(outlier_metric)
plt.plot(outlier_metric, color='blue')
# %%
