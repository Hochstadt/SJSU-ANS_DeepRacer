#Purpose is to just open the lidar messages and save them NOT
#as ROS objects, because working with ROS objects outside of ROS
#framework, ie. jupyter notebooks, is a major pain....

import pickle
import sys
import cv2
import os
import re
import numpy as np
import datetime
from sensor_msgs_py import point_cloud2

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print('Need to call as follows')
        print('python3 convert_to_data_list data_location')
    else:
        
        print('Creating numpy point clouds')
        data_dir = sys.argv[1]
        files = os.listdir(data_dir)
        for f in files:
            if 'pc2.' in f:           
                with open(os.path.join(data_dir, f), 'rb') as handle:
                    pointcloud = pickle.load(handle)
                point_list = point_cloud2.read_points_list(pointcloud)
                pl_length = len(point_list)
                point_array = np.empty((3, pl_length))
                for i, p in enumerate(point_list):
                    point_array[0, i] = p.x
                    point_array[1,i] = p.y
                    point_array[2,i] = p.z
                #Slightly adjsut the file name
                farr = f.split('.')
                new_fname = farr[0] + 'nparray.pickle'
                with open(os.path.join(data_dir, new_fname), 'wb') as handle:
                    pickle.dump(point_array, handle)
        #Now create a video using one of the images:
        YEAR = 2023
        MONTH = 4
        print('Creating debugging video')
        video_name = 'video.avi'
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

        #Run through them
        video = -1
        for f in sorted_files:
            if 'img' in f:
                with open(os.path.join(data_dir, f), 'rb') as handle:
                    images = pickle.load(handle)
                #Just take the first image
                dimg = images[0]
                height, width, layers = dimg.shape
                file_time = re.findall('\d\d', f)                
                time_tag = file_time[1] + '_'+file_time[2]+'_'+file_time[3]+'_'+file_time[4]+file_time[5]
                cv2.putText(dimg, time_tag, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                            (0, 255, 255), 2, cv2.LINE_4)
                #, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                #            (0, 255, 255), 2, cv2.LINE_4)
                if video == -1:
                    video = cv2.VideoWriter(os.path.join(data_dir, video_name), 0, 5, (width, height))

                video.write(dimg)

        cv2.destroyAllWindows()
        video.release()
        
