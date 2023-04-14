#This is an implementation I found online here:
#
#

import math
import numpy as np
from sklearn.neighbors import NearestNeighbors
import pyransac3d as pyrsc

def linesac(point_cloud, include_radius = .1):
    ransac_line = pyrsc.Line()
    cleaned_pt_cloud = np.empty((3,0))
    #p = point_cloud[:, 0]
    #iterations = 100
    for i in range(0, point_cloud.shape[1]):
        p = point_cloud[:, i]
        #get distance to all other points
        dist = np.subtract(p.reshape((3, 1)) , point_cloud)
        dist = np.sqrt(dist[0,:]**2 + dist[1, :]**2 + dist[2,:]**2)
        #Get all points that are within 1m of this point and fit
        ind = dist <= include_radius
        pot_inliers = point_cloud[:, ind]
        if pot_inliers.shape[1] > 10:
            (A, B, inliers) = ransac_line.fit(pot_inliers.transpose(),  thresh=.05, maxIteration=50)
            #print('Inlier amnt: ', inliers.shape)
            #Update new point:
            #p = pot_inliers[:, -1]
            cleaned_pt_cloud = np.append(cleaned_pt_cloud, pot_inliers[:, inliers], axis = 1)
            #Remove identical poitns
            cleaned_pt_cloud = np.unique(cleaned_pt_cloud, axis = 1)#<_ hoping the 3Dimensions are along column, 3xN
    return cleaned_pt_cloud



#Quick euc dist def
def euclidean_distance(point1, point2):
    """
    Euclidean distance between two points
    Expected input as a tuple, will return the euclidean distance
    """

    a = np.array(point1)
    b = np.array(point2)

    return np.linalg.norm(a-b, ord=2)



def point_based_matching(point_pairs):
    '''
    See paper "Robot Pose Estimation in Unknown Environments by Mathcing 2D Range Scans
    point_pairs - matched point pairs
    retur - the rotation angle and the 2D translation to be applied for matching the given pairs of points
    '''

    x_mean = 0
    y_mean = 0
    xp_mean = 0
    yp_mean = 0
    n = len(point_pairs)
    if n == 0:
        return None, None, None

    for pair in point_pairs:
        #Iterate through all point paris
        (x,y), (xp, yp) = pair
        #p - original points
        #no p - new points
        
        #Find conglomerative mean of x and y of each corresponding point
        x_mean += x
        y_mean += y
        xp_mean += xp
        yp_mean += yp

    x_mean /= n
    y_mean /= n
    xp_mean /= n
    yp_mean /= n



    s_x_xp = 0
    s_y_yp = 0
    s_x_yp = 0
    s_y_xp = 0
    for pair in point_pairs:
        #Once again iterate through all
        (x,y), (xp, yp) = pair
        #Find deviations from the mean on each, then multiply
        s_x_xp += (x - x_mean) * (xp - xp_mean)
        s_y_yp += (y - y_mean) * (yp - yp_mean)
        s_x_yp += (x - x_mean) * (yp - yp_mean)
        s_y_xp += (y - y_mean) * (xp - xp_mean)

    rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
    translation_x = xp_mean - (x_mean * math.cos(rot_angle) - y_mean*math.sin(rot_angle))
    translation_y = yp_mean - (x_mean * math.sin(rot_angle) + y_mean*math.cos(rot_angle))

    return rot_angle, translation_x, translation_y


def run_icp(reference_points, points, max_iterations = 100, distance_threshold=.3, convergence_translation_threshold=1e-3,
        convergence_rotation_threshold=1e-4, point_pairs_threshold=10, verbose=False):
    '''
        Matches one set of points to another set of points...
        refernece poitns - reference point set as numpy array (N x 2)
        points - points to be aligned, M x 2
        max_iterations - 
        distance_threshold - what constitutes a pair?
        convergence_translation_threshold: when is the translation converged (in terms of the translaiton piece)
        ditto for rotation
        point_paris_threshold: the minimum number of point pairs that should exist
        return: transformation history as a list of numpy arrays contains R and T for each iteration as well as teh newly aligned
        points
    '''

    total_translation = np.zeros((2,1))
    total_rot = np.identity(2)
    transformation_history = []
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)
    for iter_num in range(max_iterations):
        if verbose:
            print('Iterations', iter_num)
        closest_point_pairs = [] #point correspondences
        #find the closest neighbors using a kd_tree NN approach
        distances, indices = nbrs.kneighbors(points)
        for nn_index in range(len(distances)):
            if distances[nn_index][0] < distance_threshold:
                #MATCH~
                closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))

        if verbose:
            print('number of pairs found: ', len(closest_point_pairs))
        if len(closest_point_pairs) < point_pairs_threshold:
            if verbose:
                print('No better solution can be found, not too many point pairs')
            break
        
        #Get the traslation and rotatio of the point correspondences
        closest_rot_angle, closest_translation_x, closest_translation_y = point_based_matching(closest_point_pairs)
        if closest_rot_angle is not None:
            if verbose:
                print('Rotation: ', math.degrees(closest_rot_angle), 'degrees')
                print('Translation: ', closest_translation_x, closest_translation_y)
        if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
            if verbose:
                print('no better solution can be found')
            break

        #Transform points
        c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
        rot = np.array([[c, -s],[s, c]])
        total_rot = np.matmul(rot, total_rot)

        aligned_points = np.dot(points, rot.T)
        t1_0 = np.array([[closest_translation_x],[closest_translation_y]])
        if total_translation[0] == 0:
            total_translation = t1_0
        else:
            total_translation = t1_0 + np.matmul(rot, total_translation)

        
        aligned_points[:,0] += closest_translation_x
        aligned_points[:,1] += closest_translation_y

        #Update for next iteration
        points = aligned_points
        #update transofmration history
        transformation_history.append(np.hstack((rot, np.array([[closest_translation_x], [closest_translation_y]]))))

        #check convergence
        if abs(closest_rot_angle) < convergence_rotation_threshold \
            and abs(closest_translation_x) < convergence_translation_threshold \
            and abs(closest_translation_y) < convergence_translation_threshold:
            if verbose:
                print('Converged')
            break

    return total_rot, total_translation, aligned_points
        


