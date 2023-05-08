#%%
import open3d as o3d
import pyransac3d as pyrsc
import pickle
import sys
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
#%%
#from deepracer_interfaces_pkg.msg import CameraMsg
#from cv_bridge import CvBridge
import cv2
import os
import re
import numpy as np
import datetime
import icp
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull, convex_hull_plot_2d



if __name__ == '__main__':
    if len(sys.argv) < 1:
        print('Need to call as follows')
        print('python3 icp_map_builder data_location')
    else:

        #Variables
        data_dir = sys.argv[1]
        #data_dir = 'data/slowis_steady'
        MAP_FILE_NAME = 'map_file.pickle'
        MAP_PCD_NAME = 'map_file.pcd'
        OCCUPANCY_PNG = 'occupancy.png'
        OCCUPANCY_YAML = 'occupancy.yaml'
        YEAR = 2022
        MONTH = 5
        #Helps set the starting frame
        start_index = 0
        INFO_EVERY = 10
        DISPLAY_EVERY = 50
        STOP_AFTER = 15000
        #the value to help with outlier rejection
        DVALUE = .1
        #will not add points if outside this 
        DISTANCE_THRESHOLD = .5
        #Trimming parameters
        TRIM_EVERY = 100
        TRIM_RADIUS = .05
        TRIM_CONSISTENCY = 25
        #Flags
        bDisplay = True
        bDebug = False
        bOutlierRej = False

        ##Data collection
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

        #Identify reference frame
        tmp_sorted_files = sorted_files[start_index:-1]
        for f in tmp_sorted_files:
            if 'nparray' in f:
                ref_id = start_index
                break
            start_index+=1

        with open(os.path.join(data_dir, sorted_files[ref_id]), 'rb') as handle:
            ref_pointcloud = pickle.load(handle)
        ref_name = sorted_files[ref_id]
        ref_2dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:]]).transpose()
        ref_3dof = np.array([ref_pointcloud[0,:], ref_pointcloud[1,:], ref_pointcloud[2,:]])

        clean_ref_3dof = icp.linesac(ref_3dof, include_radius=1)
        sorted_times = sorted_times[ref_id + 1:-1]
        sorted_files = sorted_files[ref_id + 1:-1]
        #intermediate map
        pc_map = clean_ref_3dof[0:2, :].transpose()
        #full map
        full_pc_map = []
        #state estimators
        Rest_obs = np.identity(2)
        test_obs = np.zeros((2,1))
        #track metrics throughout
        translation = []
        rotation_angle  = []
        sensor_loc = []
        time = []
        #count
        c = 0
        total_files = sorted_files.shape[0]

        for f in sorted_files:   
            if c % INFO_EVERY == 0 and c > 1:
                print('Iteration: ', c, ' out of ', total_files, '. On file: ', f)

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
                (Rmap_est, tmap_est, dont_use, outlier_mask) = icp.run_icp(pc_map, est_pc_map.transpose(), DVALUE, 
                                                                        distance_threshold=DISTANCE_THRESHOLD, verbose=False)        

                #Print the rotation angle and translation        
                rot_angle = math.acos(Rmap_est[0, 0])
                rot_angle_check = math.asin(Rmap_est[1,0])
                if bDebug == True:
                    
                    print('Rotation is: ', math.degrees(rot_angle))        
                    print('Translation is: <', tmap_est[0], ',', tmap_est[1], '>')
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

                if c > STOP_AFTER:
                    break

            if c % TRIM_EVERY == 0 and c > 1:
                pc_map = icp.trimPointsV2(pc_map, scan_radius = TRIM_RADIUS, POINT_THRESH=TRIM_CONSISTENCY)
                #Add the pc map to the full one:
                if np.any(full_pc_map) == False:
                    full_pc_map = np.copy(pc_map)
                else:
                    full_pc_map = np.vstack((full_pc_map, pc_map))
                pc_map = np.copy(full_pc_map)

            if c % DISPLAY_EVERY == 0 and c > 1 and bDisplay:
                plt.scatter(pc_map[:,0], pc_map[:,1], color='blue')
                if not np.any(full_pc_map) == False:
                    plt.scatter(full_pc_map[:,0], full_pc_map[:,1], color='green')
                tmp_sensor = np.array(sensor_loc)
                tmp_sensor = tmp_sensor.reshape((tmp_sensor.shape[0], tmp_sensor.shape[1]))
                plt.scatter(tmp_sensor[:,0], tmp_sensor[:,1], color='magenta')
                plt.axis('equal')
                fname = str(c)+'_display.png'
                fname = os.path.join(data_dir, fname)
                plt.savefig(fname)
                

            c+=1






        if bDisplay:
            plt.scatter(full_pc_map[:,0], full_pc_map[:,1], color='blue')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.axis('equal')
            fname = 'final_map.png'
            fname = os.path.join(data_dir, fname)
            plt.savefig(fname)
            plt.show()

        # NStart generating the occ map
        # Specify size and resolution of map
        gridSizeMeters = 16
        res = .05


        # Calculate grid size and center location
        gridSize = gridSizeMeters/res
        center = np.array([(gridSize-1)/2, (gridSize-1)/2])

        #Save the finalized map as pickle file
        pts = []
        for i in range(full_pc_map.shape[0]):
            x = full_pc_map[i, 0]
            y = full_pc_map[i, 1]
            z = 0
            pt3D = np.array([x, y, z])
            pts.append(pt3D)

        pts = np.array(pts)

        #Add pts into the pcd file and save, then we can do some other stuff too
        sample_pcd_data = o3d.data.PCDPointCloud()
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(os.path.join(data_dir,MAP_PCD_NAME), pcd)


        r = R.from_euler('z', -np.pi/2)
        rotated_pts  = np.matmul(r.as_matrix(), pts.transpose()) + (res * np.array([[center[0], center[1], 0]]).transpose())
        rotated_pts = rotated_pts.transpose()
        fname = os.path.join(data_dir, MAP_FILE_NAME)
        with open(fname, 'wb') as handle:
            pickle.dump(rotated_pts, handle)

        # Initialize Occupancy Map
        occGrid = np.zeros((int(gridSize),int(gridSize),3), dtype=np.uint8)
        #According to Curtis need to flip
        occGrid = np.rot90(occGrid, k=-1, axes=(0,1))

        # Loop through each point in point cloud
        num_pts = pts.shape
        for i in range(0, num_pts[0]):
            # Set location in Occupancy Map to occupied
            location = pts[i,:2]/res + center
            #location = pts[i,:2]/res
            occGrid[int(location[0]),int(location[1])] = [255,255,255]

        import yaml
        from PIL import Image
        # Create Occupancy Map YAML Strucutre
        occupancy_grid = dict()
        occupancy_grid['resolution'] = res
        occupancy_grid['origin'] = [0.0, 0.0, 0.0]
        occupancy_grid['negate'] = 0
        occupancy_grid['occupied_thresh'] = 1.0
        occupancy_grid['free_thresh'] = 0.0
        imgname = OCCUPANCY_PNG
        occupancy_grid['image'] = imgname
        occupancy_grid['mode'] = "scale"

        # Write Occupancy Map YAML
        with open(os.path.join(data_dir, OCCUPANCY_YAML), 'w') as fp:
            yaml.dump(occupancy_grid, fp)

        # Save Occupancy Map image
        img = Image.fromarray(occGrid)
        img.save(os.path.join(data_dir, imgname))
