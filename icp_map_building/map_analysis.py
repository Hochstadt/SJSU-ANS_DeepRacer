#%%
import numpy as np
import cv2
import os
import re
import numpy as np
import datetime
import icp
import math
import pickle
import matplotlib.pyplot as plt


# %%
# Load the map
analysis_map_file = 'data/slowish_okay2/map_file.pickle'

with open(analysis_map_file, 'rb') as handle:
    pts = pickle.load(handle)
pts = pts.transpose()
plt.plot(pts[0,:], pts[1,:], '.', color='green')
plt.axis('equal')
plt.show()
# %%
#Define appx locations to extract in the map for analysis
'''
fast_okay
box = [.3, .8]
box_radius = .4
table = [-1.25, -.3]
table_radius = .7
wood = [-1, 1.75]
wood_radius  = .5
thin_cylinder = [.5, -1.1]
cylinder_radius = .1
'''
#slowish_okay2
box = [0, -1.5]
box_radius = .8
table = [-2, -2]
table_radius = .7
wood = [-1, -.25]
wood_radius  = .5
thin_cylinder = [-.3, -3.5]
cylinder_radius = .1

#Now go extract
object = box
object_radius = box_radius
def extract_object(object, object_radius):
    tl_x = object[0] - object_radius
    tl_y = object[1] - object_radius
    tr_x = object[0] + object_radius
    tr_y = object[1] - object_radius
    br_x = object[0] + object_radius
    br_y = object[1] + object_radius
    bl_x = object[0] - object_radius
    bl_y = object[1] + object_radius

    np_pts = np.array(pts)
    valid_pts_x = np.logical_and(np_pts[0,:] < tr_x, np_pts[0,:] > tl_x)
    valid_pts_y = np.logical_and(np_pts[1,:] < br_y, np_pts[1,:] > tr_y)
    valid_pts_i = np.logical_and(valid_pts_x, valid_pts_y)
    #Define the bounding box
    valid_pts = np_pts[:, valid_pts_i]
    min_x = np.min(valid_pts[0,:])
    max_x = np.max(valid_pts[0,:])
    min_y = np.min(valid_pts[1,:])
    max_y = np.max(valid_pts[1,:])
    bb = np.array([[min_x, min_y],[max_x, min_y], [max_x, max_y],[min_x, max_y]]).transpose()
    
    return valid_pts, bb

font = {'family' : 'normal',
        'weight' : 'bold', 
        'size' : 22}
plt.rc('font', **font)
msize = 10
plt.plot(pts[0,:], pts[1,:], '.', color='black', markersize=msize)
#box
box_pts, box_bb = extract_object(box, box_radius)
plt.plot(box_pts[0,:], box_pts[1,:], '.', color='red', markersize=msize)
#table
table_pts, table_bb = extract_object(table, table_radius)
plt.plot(table_pts[0,:], table_pts[1,:], '.', color='blue', markersize=msize)
#4x4
wood_pts, wood_bb = extract_object(wood, wood_radius)
plt.plot(wood_pts[0,:], wood_pts[1,:], '.', color='magenta', markersize=msize)
#cylinder
cyl_pts, cyl_bb = extract_object(thin_cylinder, cylinder_radius)
plt.plot(cyl_pts[0,:], cyl_pts[1,:], '.', color='green', markersize=msize)
plt.title('Test Area')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid('on')
plt.axis('equal')
plt.show()


# %% 
#Plot close-ups
msize = 25
plt.plot(box_pts[0,:], box_pts[1,:], '.', color='red', markersize=msize)
plt.title('Cardboard Box')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid('on')
plt.axis('equal')
plt.show()
plt.plot(table_pts[0,:], table_pts[1,:], '.', color='blue', markersize=msize)
plt.title('Table')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid('on')
plt.axis('equal')
plt.show()
plt.plot(wood_pts[0,:], wood_pts[1,:], '.', color='magenta', markersize=msize)
plt.title('4" x 4" Wood Block')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid('on')
plt.axis('equal')
plt.show()
plt.plot(cyl_pts[0,:], cyl_pts[1,:], '.', color='green', markersize=msize)
plt.title('Small Cylinder')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.grid('on')
plt.axis('equal')
plt.show()
#%%
#Now decompose each object and try to assess dimensionality
#
from scipy.spatial import ConvexHull, convex_hull_plot_2d
def box_analysis(object_pts, bb, true_w, true_l):

    plt.plot(object_pts[0,:], object_pts[1,:], '.', color='green')
    hull = ConvexHull(object_pts.transpose())
    convex_hull_plot_2d(hull)
    plt.grid('on')
    plt.plot(bb[0, :], bb[1,:], '.', color='red')

    bb_w = (bb[0, 1] - bb[0, 0])
    bb_h = (bb[1, 2] - bb[1, 1])
    print('Bounding box dimensions: ', bb_w, 'x', bb_h)
    appx_box_area = (bb[0, 1] - bb[0, 0]) * (bb[1, 2] - bb[1, 1])
    spatial_area = hull.volume
    #box iou calculated via, intersection as sub area and union as greater area
    box_iou = spatial_area/appx_box_area

    hull_perimeter = hull.area
    hull_area = hull.volume
    bb_perimeter = bb_w * 2 + bb_h * 2
    actual_perimeter = true_w * 2 + true_l*2
    actual_area = true_w * true_l
    print('BB Height: ', bb_h, 'BB Width: ', bb_w)
    print('Actual perimeter: ', actual_perimeter, 'measured perimeter', hull_perimeter)
    print('Actual area: ', actual_area, 'measured area', hull_area)
    print('box iou, ', box_iou)

box_analysis(box_pts, box_bb, 2*12/39.97, 17.25/39.97)

# %%
box_analysis(wood_pts, wood_bb, 3.75/39.97, 3.75/39.97)
# %%
box_analysis(table_pts, table_bb, 24.25/39.97, 24.25/39.97)
# %%
def circle_analysis(object_pts, bb, true_d):

    plt.plot(object_pts[0,:], object_pts[1,:], '.', color='green')
    hull = ConvexHull(object_pts.transpose())
    convex_hull_plot_2d(hull)
    plt.grid('on')
    plt.plot(bb[0, :], bb[1,:], '.', color='red')

    bb_w = (bb[0, 1] - bb[0, 0])
    bb_h = (bb[1, 2] - bb[1, 1])
    print('Bounding box dimensions give circle with diameter of: ', bb_w)
    appx_box_area = (bb[0, 1] - bb[0, 0]) * (bb[1, 2] - bb[1, 1])
    spatial_area = hull.volume
    #box iou calculated via, intersection as sub area and union as greater area
    box_iou = spatial_area/appx_box_area

    hull_perimeter = hull.area
    hull_area = hull.volume
    bb_perimeter = bb_w * 2 + bb_h * 2
    actual_perimeter = 2*np.pi * (true_d/2)
    actual_area = np.pi *(true_d/2)**2
    print('Actual perimeter: ', actual_perimeter, 'measured perimeter', hull_perimeter)
    print('Actual area: ', actual_area, 'measured area', hull_area)
    print('box iou, ', box_iou)

circle_analysis(cyl_pts, cyl_bb, 4.25/39.97)
# %%
