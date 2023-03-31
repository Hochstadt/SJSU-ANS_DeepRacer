import pickle
#from sensor_msgs.msg import LaserScan
#from deepracer_interfaces_pkg.msg import CameraMsg
#from cv_bridge import CvBridge
import cv2

fname = '/home/taylor/SJSU-ANS_DeepRacer/2023_03_31_16_23_38/31_16_24_43_467857_img.pickle'
with open(fname, 'rb') as handle:
    mydata = pickle.load(handle)

img1 = mydata[0]
img2 = mydata[1]

#bridge = CvBridge()
#cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

cv2.imshow('testimg', img1 - img1)
cv2.waitKey()

#Stopping for today.
#Work done - integrated Curtis' lidar thing, able to save images. Now want to setup
#for the actual data collection. Where the car is being driven around and data is being stored
#want to store lidar point clouds and the img type. Can use this to potenitally do the conversion
#before storing the images.

#Then we have datasets we can start to work with!

