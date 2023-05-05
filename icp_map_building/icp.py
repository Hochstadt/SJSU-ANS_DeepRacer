#This is an implementation I found online here:
#
#

import math
import numpy as np
from sklearn.neighbors import NearestNeighbors
import pyransac3d as pyrsc
import numpy as np # Scientific computing library for Python
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def trimPointsV2(point_cloud, scan_radius = .1, POINT_THRESH=2):
    #Define grid to nonmax on
    max_x = np.max(point_cloud[:, 0])
    max_y = np.max(point_cloud[:, 1])
    min_x = np.min(point_cloud[:, 0])
    min_y = np.min(point_cloud[:, 1])
    
    #Create the rang eusing arange
    xgrid = np.arange(min_x, max_x, scan_radius)
    ygrid = np.arange(min_y, max_y, scan_radius)
    new_pts = []
    c = 0
    for x in xgrid:
        #if c % 10 == 0:
        #    print('iteration: ', c, 'out of ', len(xgrid))
        back_x = x - scan_radius/2
        for_x = x + scan_radius/2
        x_i = np.logical_and(point_cloud[:,0] >= back_x, point_cloud[:,0] < for_x)
        c+=1
        if sum(x_i) >= POINT_THRESH:
            for y in ygrid:
                back_y = y - scan_radius/2
                for_y = y + scan_radius/2
                #Capture all points within the sqaure
                y_i = np.logical_and(point_cloud[:, 1] >= back_y, point_cloud[:,1] < for_y)
                log_i = np.logical_and(x_i, y_i)
                captured_points = point_cloud[log_i, :]
                if captured_points.shape[0] >= POINT_THRESH:  
                    #print('Captured ', captured_points.shape[0], 'points')                  
                    new_pts.append(np.mean(captured_points, axis=0))
            
    return np.array(new_pts)

def trimPoints(point_cloud, include_radius = .01):
    h, w = point_cloud.shape
    if h > w:
        point_cloud = point_cloud.transpose()

    trimmed_pt_cloud =np.empty((2,0))
    include_ind = np.ones((point_cloud.shape[1],))
    for i in range(0, point_cloud.shape[1]):
        p = point_cloud[:,i]
        if include_ind[i] == 1:
            #Evaluate
            dist = np.subtract(p.reshape((2, 1)) , point_cloud)
            dist = np.sqrt(dist[0,:]**2 + dist[1, :]**2)
            ind = dist <= include_radius
            #Don't look at all neighbors within radius
            include_ind[ind] = 0
            trimmed_pt_cloud = np.append(trimmed_pt_cloud, p.reshape((2,1)), axis=1)
    
    if h > w:
        #flip back
        point_cloud = point_cloud.transpose()
        trimmed_pt_cloud = trimmed_pt_cloud.transpose()
    return trimmed_pt_cloud

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

def refine_point_pairs(point_pairs, D=-1):

    n = len(point_pairs)
    if n == 0:
        print('Yopu done messed up')

    #Find mean
    dist_mean = 0
    max_dist = 0
    for pair in point_pairs:
        (x,y), (xp, yp) = pair
        dist = euclidean_distance([x,y], [xp, yp])
        dist_mean += dist
        if dist > max_dist:
            max_dist = dist

    #set eta as 75% of the Dmax
    eta = max_dist * .75

    dist_mean /= n
    #Find std
    dist_std = 0
    for pair in point_pairs:
        (x,y), (xp, yp) = pair
        dist_std += (euclidean_distance([x,y], [xp, yp]) - dist_mean)**2

    dist_std = math.sqrt(1/n * dist_std)
    if D == -1:
        #This is kind of the idle time, when we should have perfect matches. Thus D will be equal
        #to the mean
        D = dist_mean
        Dmax = dist_mean + 3 * dist_std
        return point_pairs, Dmax, D
    else:
        
        if dist_mean < D:
            #Good match
            Dmax = dist_mean + 3 * dist_std
        elif dist_mean < 3 * D:
            #okay match
            Dmax = dist_mean + 2 * dist_std
        elif dist_mean < 6 * D:
            #not so good match
            Dmax = dist_mean + dist_std
        else:
            #bad match
            Dmax = eta

        #Now run through pairs one more time and expel any pairs that have
        #distances larger than Dmax
        to_pop_index = []
        to_keep_index = []
        
        for indx, pair in enumerate(point_pairs):
            (x,y), (xp, yp) = pair
            if euclidean_distance([x,y], [xp, yp]) > Dmax:
                to_pop_index.append(indx)
            else:
                to_keep_index.append(indx)        
        #pop in reverse so that indexing doesn't get off
        #(ie. biggest to smallest)
        for indx in reversed(to_pop_index):            
            point_pairs.pop(indx)
        return point_pairs, to_keep_index, to_pop_index


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
    std_dist = 0
    for pair in point_pairs:
        #Once again iterate through all
        (x,y), (xp, yp) = pair
        #Find deviations from the mean on each, then multiply
        s_x_xp += (x - x_mean) * (xp - xp_mean)
        s_y_yp += (y - y_mean) * (yp - yp_mean)
        s_x_yp += (x - x_mean) * (yp - yp_mean)
        s_y_xp += (y - y_mean) * (xp - xp_mean)

    #s_x_yp  = how much - rotation we want in positive direction (up to 90 degrees)
    #   this is the global correlation between the map's x coordinates and the incoming
    #   point's y coordinates. If this is strong it means the incoming pointc loud y's coordinates
    #   most closely correlate to the map's x coordinates, so do a positve rotation
    #s_y_xp = how much rotation we want in negative rotation
    #   as before htis is how well the map's y coordniates are aligned with incoming
    #   point clouds x coordinates. 
    #s_x_xp and s_y_yp - this is a measure of how well the alignments are correlated
    #   for perfect alignment this value is the maximum it could be, and htus the roation
    #   angle output will be near 0


    rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
    #To get the translation all we do is rotate the map to get to the incoming point and then apply
    # the mean offset. As long as the point clouds are mostly the same this method is sound
    translation_x = xp_mean - (x_mean * math.cos(rot_angle) - y_mean*math.sin(rot_angle))
    translation_y = yp_mean - (x_mean * math.sin(rot_angle) + y_mean*math.cos(rot_angle))

    return rot_angle, translation_x, translation_y


def run_icp(reference_points, points, D, max_iterations = 100, distance_threshold=.3, convergence_translation_threshold=1e-3,
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
    bRefinePoints = True
    total_translation = np.zeros((2,1))
    total_rot = np.identity(2)
    transformation_history = []
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)
    outlier_indices = np.array([],dtype=int)
    bool_mask = np.ones(points.shape[0], dtype=bool)

    for iter_num in range(max_iterations):
        if verbose:
            print('Iterations', iter_num)
        closest_point_pairs = [] #point correspondences
        tmp_inlier_indices = []
        tmp_outlier_indices = []
        #find the closest neighbors using a kd_tree NN approach
        distances, indices = nbrs.kneighbors(points)
        for nn_index in range(len(distances)):
            if distances[nn_index][0] < distance_threshold:
                #MATCH~
                closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))
                #Keep track of best points...
                tmp_inlier_indices.append(nn_index)            
            else:
                tmp_outlier_indices.append(nn_index)
                
        
        #points = points[np.array(tmp_inlier_indices),:]
        #Once an outlier always and outlier...
        #outlier_indices = np.union1d(outlier_indices, np.array(tmp_outlier_indices))
        if verbose:
            print('number of pairs found: ', len(closest_point_pairs))
        if len(closest_point_pairs) < point_pairs_threshold:
            if verbose:
                print('No better solution can be found, not too many point pairs')
            break
        #Refine the point pairs using section 3.3 of Zhang (1994)

        if bRefinePoints == True:
            (closest_point_pairs, tmp_inlier_indices, tmp_outlier_indices) = refine_point_pairs(closest_point_pairs, D)
            #Degrade points as well:
            #points = points[tmp_inlier_indices, :]
            #inlier_indices = np.union1d(inlier_indices, np.array(tmp_inlier_indices))
            outlier_indices = np.union1d(outlier_indices, np.array(tmp_outlier_indices))


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
    #Create mask out of outlier_indices
    bool_mask[outlier_indices.astype(int)] = False
    return total_rot, total_translation, aligned_points, bool_mask
        


