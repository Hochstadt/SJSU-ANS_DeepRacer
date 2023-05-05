#ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import ros2_numpy

#Python
import os
import math
import threading
import numpy as np
import time
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class Localization(Node):
        
        envMap                 = o3d.geometry.PointCloud()
        envMapSmall            = o3d.geometry.PointCloud()
        solutionFound          = False
        currentState           = np.identity(4)
        goalState              = np.identity(4)
        Tcar_lidar             = np.array([[-1.0, 0.0, 0.0, 0.0],[0.0, -1.0, 0.0, 0.0],[0.0, 0.0, 1.0, 0.0],[0.0, 0.0, 0.0, 1.0]])
        downsampleMap          = True
        downsampleValue        = 0.25
        coarse_fit_thresh      = 0.25
        fine_fit_thresh        = 0.25
        icp_max_corr_dist      = 1.0
        icp_max_iters          = 100
        min_search_x           = -10.0
        max_search_x           = 2.0
        delta_search_x         = 5
        min_search_y           = -2.0
        max_search_y           = 10.0
        delta_search_y         = 5 
        delta_search_ang       = 5
        rotationCheckThreshold = 0.017
        positionCheckThrehsold = 0.100

        def __init__(self):
            super().__init__('localization')

            self.get_logger().info("Started")

            # Read in configurable parameters
            self.load_config()

            # Establish QoS parameters
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
                )

            # Create Map Load message subscriber
            self.mMapSub = self.create_subscription(
                PointCloud2,
                "/ans_services/map_pt_msg",
                self.store_map,
                qos_profile=qos_profile)
                
            # Create LiDAR scan message subscriber
            self.mLidarPtCloudSub = self.create_subscription(
                PointCloud2,
                "/lidar_scan_acq/lidar_pt_msg",
                self.estimate_pose,
                qos_profile=qos_profile)
                
            # Create Goal State message subscriber
            self.mGoalStateSub = self.create_subscription(
                PoseStamped,
                "/goal_pose",
                self.store_goal_state,
                qos_profile=qos_profile)
                
            # Create Localization Pose Estimate publisher
            self.mPoseEstPub = self.create_publisher(
                PoseStamped,
                "/localization/pose",
                1)

            # Create Localization Pose Estimate publisher
            self.mAlignedLidarPtCloudPub = self.create_publisher(
                PointCloud2,
                "/localization/aligned_lidar_pt_msg",
                1)
           
            # Create atGoalState publisher
            self.mAtGoalStatePub = self.create_publisher(
                Bool,
                "/localization/at_goal_state_msg",
                1)
           
            # Create solutionFound publisher
            self.mSolutionFoundPub = self.create_publisher(
                Bool,
                "/localization/solution_found",
                1)

        def load_config(self):

            # Declare input variables
            self.declare_parameter("downsampleMap",  self.downsampleMap)
            self.declare_parameter("downsampleValue",  self.downsampleValue)
            self.declare_parameter("coarse_fit_thresh",  self.coarse_fit_thresh)
            self.declare_parameter("fine_fit_thresh",  self.fine_fit_thresh)
            self.declare_parameter("icp_max_corr_dist",  self.icp_max_corr_dist)
            self.declare_parameter("icp_max_iters",  self.icp_max_iters)
            self.declare_parameter("min_search_x",  self.min_search_x)
            self.declare_parameter("max_search_x",  self.max_search_x)
            self.declare_parameter("delta_search_x",  self.delta_search_x)
            self.declare_parameter("min_search_y",  self.min_search_y)
            self.declare_parameter("max_search_y",  self.max_search_y)
            self.declare_parameter("delta_search_y",  self.delta_search_y)
            self.declare_parameter("delta_search_ang",  self.delta_search_ang)
            self.declare_parameter("rotationCheckThreshold",  self.rotationCheckThreshold)
            self.declare_parameter("positionCheckThrehsold",  self.positionCheckThrehsold)

            # Set input variables
            self.downsampleMap = self.get_parameter("downsampleMap").get_parameter_value().integer_value
            self.downsampleValue = self.get_parameter("downsampleValue").get_parameter_value().double_value
            self.coarse_fit_thresh = self.get_parameter("coarse_fit_thresh").get_parameter_value().double_value
            self.fine_fit_thresh = self.get_parameter("fine_fit_thresh").get_parameter_value().double_value
            self.icp_max_corr_dist = self.get_parameter("icp_max_corr_dist").get_parameter_value().double_value
            self.icp_max_iters = self.get_parameter("icp_max_iters").get_parameter_value().integer_value
            self.min_search_x = self.get_parameter("min_search_x").get_parameter_value().double_value
            self.max_search_x = self.get_parameter("max_search_x").get_parameter_value().double_value
            self.delta_search_x = self.get_parameter("delta_search_x").get_parameter_value().integer_value
            self.min_search_y = self.get_parameter("min_search_y").get_parameter_value().double_value
            self.max_search_y = self.get_parameter("max_search_y").get_parameter_value().double_value
            self.delta_search_y = self.get_parameter("delta_search_y").get_parameter_value().integer_value
            self.delta_search_ang = self.get_parameter("delta_search_ang").get_parameter_value().integer_value
            self.rotationCheckThreshold = self.get_parameter("rotationCheckThreshold").get_parameter_value().double_value
            self.ositionCheckThrehsold = self.get_parameter("positionCheckThrehsold").get_parameter_value().double_value

        def store_map(self, msg):
            
            # convert ros msg to open3d
            self.envMap = self.convert_to_pc_o3d(msg)

            # Downsample map
            self.envMapSmall = self.envMap.voxel_down_sample(voxel_size=self.downsampleValue)
            
            # Inform that message was received
            self.get_logger().info('Map has been received')

        def store_goal_state(self, goalMsg):
                   
            # Inform that message was received
            self.get_logger().info("Goal State received")

            # Initialize Goal State Pose
            self.goalState = np.identity(4)
            
            # Store Position
            self.goalState[0,3] = goalMsg.pose.position.x
            self.goalState[1,3] = goalMsg.pose.position.z
            self.goalState[2,3] = goalMsg.pose.position.x

            # Calculate DCM
            q = np.array([goalMsg.pose.orientation.x, goalMsg.pose.orientation.y, goalMsg.pose.orientation.z, goalMsg.pose.orientation.w])
            rot = R.from_quat(q)
            dcm = rot.as_matrix()

            # Store quaternion in pose msg
            self.goalState[0:3,0:3] = dcm


        def estimate_pose(self, lidarMsg):
            
            t0 = time.time()

            #Check if enought ime has apssed
            if (not self.envMap.is_empty()):
                
                # Convert LiDAR scan to open3d point cloud
                lidarScan = self.convert_to_pc_o3d(lidarMsg)

                # Define configurable parameters
                icpType = o3d.pipelines.registration.TransformationEstimationPointToPoint(False)
                iterations = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = self.icp_max_iters)

                # Decide which map to use
                if self.icp_max_corr_dist:
                    Map = self.envMapSmall
                else:
                    Map = self.envMap

                # Perform pose search if no a priori knowledge
                if not self.solutionFound:
                    self.currentState = self.searchForInitialPose(lidarScan, Map, icpType, iterations)
                    poseMsg = np.matmul(self.currentState, self.Tcar_lidar)

                # Perform ICP with previous pose information
                else: 
                    results = o3d.pipelines.registration.registration_icp(lidarScan, Map, self.icp_max_corr_dist, self.currentState, icpType, iterations)

                    # Extract results
                    trans = np.array(results.transformation)
                    fitnessScore = results.inlier_rmse

                    # Store and confirm pose if quality solution produced
                    if fitnessScore < self.fine_fit_thresh:
                        # Get Pose Estimate
                        self.currentState = trans
                        poseMsg = np.matmul(self.currentState, self.Tcar_lidar)
                        self.solutionFound = True
                    # Indicate no knowledge if low quality solution produced
                    else:
                        self.currentState = np.identity(4)
                        poseMsg = np.identity(4)
                        self.solutionFound = False 
                 
                # Publish Pose
                self.publish_pose_estimate(poseMsg, lidarMsg)
                
                # Publish Aligned POint CLoud
                self.publish_aligned_cloud(self.currentState, lidarScan)

                # Check Goal State
                self.at_goal_state(poseMsg)

                # Publish Solution State
                solutionFoundMsg = Bool()
                solutionFoundMsg.data = self.solutionFound
                self.mSolutionFoundPub.publish(solutionFoundMsg)

                delta_time =  (time.time() - t0)*10**3
                self.get_logger().info(f'elapsed time: {delta_time} ms')     


        def searchForInitialPose(self, Scan, Map, icpType, iterations):

            # Initialize variables
            fitScore = float('inf')
            pose = np.identity(4)

            # Loop through each position and angle combination
            t0 = time.time()
            for x in range (0, self.delta_search_x):
                for y in range (0, self.delta_search_y):
                        for theta in range(0,self.delta_search_ang):
                            
                            # Define pose conditions
                            pos_x = self.min_search_x + x * ((self.max_search_x - self.min_search_x) / self.delta_search_x)
                            pos_y = self.min_search_y  + y * ((self.max_search_y - self.min_search_y) / self.delta_search_y)
                            ang = theta*(2*np.pi/self.delta_search_ang)

                            # Construct pose
                            initial_T = np.array([[np.cos(ang),-np.sin(ang),0,pos_x],[np.sin(ang),np.cos(ang),0,pos_y],[0,0,1,0],[0,0,0,1]])

                            # Perform ICP
                            results = o3d.pipelines.registration.registration_icp(Scan, Map, self.icp_max_corr_dist, initial_T, icpType, iterations)

                            # Extract results
                            trans = np.array(results.transformation)
                            fitnessScore = results.inlier_rmse
                            self.get_logger().info('X: %f' % pos_x)
                            self.get_logger().info('Y: %f' % pos_y)
                            self.get_logger().info('ANG: %f' % ang)
                            self.get_logger().info(f'Fit: {fitnessScore}')

                            # Save pose estimate if lowest fitness seen
                            if fitnessScore < fitScore and fitnessScore > 0:

                                # Save pose estimate if within map extents
                                if trans[0,3] < self.max_search_x and trans[0,3] > self.min_search_x and trans[1,3] < self.max_search_y and trans[1,3] > self.min_search_y:

                                    pose = trans
                                    fitScore = fitnessScore

            # Save pose if fitness below threshold
            if fitScore < self.coarse_fit_thresh:
                self.solutionFound = True
                return pose

            # Return nothing if no good solution found
            else:
                self.solutionFound = False
                return pose


        def convert_to_pc_o3d(self, msg):

            # Get cloud data from ROS msg
            field_names=[field.name for field in msg.fields]
            cloud_data = list(pc2.read_points(msg, skip_nans=True, field_names = field_names))

            # Initialize point cloud
            open3d_cloud = o3d.geometry.PointCloud()

            # Populate point cloud
            if 'intensity' in field_names:
                xyz = [(x,y,z) for x,y,z,intensity  in cloud_data ] # get xyz
            else:
                xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

            return open3d_cloud
        

        def publish_pose_estimate(self, pose, scanMsg):

            # Initialize Pose Msg
            ros_pose = PoseStamped()

            # Store position in pose msg
            ros_pose.pose.position.x = pose[0,3]
            ros_pose.pose.position.y = pose[1,3]
            ros_pose.pose.position.z = pose[2,3]

            # Calculate quaternion
            rot = R.from_matrix(pose[0:3,0:3])
            q = rot.as_quat()

            # Store quaternion in pose msg
            ros_pose.pose.orientation.x = q[0]
            ros_pose.pose.orientation.y = q[1]
            ros_pose.pose.orientation.z = q[2]
            ros_pose.pose.orientation.w = q[3]

            # Create header message
            ros_pose.header.frame_id = "odom"
            ros_pose.header.stamp.sec = scanMsg.header.stamp.sec
            ros_pose.header.stamp.nanosec = scanMsg.header.stamp.nanosec

            # Publish Message
            self.mPoseEstPub.publish(ros_pose)


        def publish_aligned_cloud(self, pose, scan):

            # Get R and t from pose
            R_pose = pose[0:3,0:3]
            t_pose= pose[0:3,3]

            # Store point in array
            pointCloud = np.asarray(scan.points)

            # Align point cloud
            pointCloudAligned=[]
            for n in range(0,pointCloud.shape[0]):
                pointCloudAligned.append(np.matmul(R_pose, pointCloud[n,0:3]) + t_pose)
            pointCloudAligned=np.array(pointCloudAligned)
            data = np.zeros(pointCloud.shape[0], dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32)])
            data['x'] = pointCloudAligned[:,0]
            data['y'] = pointCloudAligned[:,1]
            data['z'] = pointCloudAligned[:,2]

            # Convert and publish point cloud msg
            #alignedCloudMsg = ros2_numpy.registry.msgify(PointCloud2, pointCloudAligned)
            alignedCloudMsg = ros2_numpy.point_cloud2.array_to_pointcloud2(data,frame_id='odom')
            self.mAlignedLidarPtCloudPub.publish(alignedCloudMsg)

        
        def at_goal_state(self, pose):

            # Get Transform from Current State and Goal State
            Tgoal_current = np.matmul(np.linalg.inv(self.goalState), pose)
            
            # Initialize message
            atGoalStateMsg = Bool()
            
            # Check if current state within threshold of goal
            if (abs(Tgoal_current[0,1]) < self.rotationCheckThreshold and np.sqrt(Tgoal_current[0,3]**2 + Tgoal_current[1,3]**2 + Tgoal_current[2,3]**2) < self.positionCheckThrehsold):
            	# set goal state flag
            	atGoalStateMsg.data = True
            else:
            	# set goal state flag
            	atGoalStateMsg.data = False
            
            # Publish goal state message
            self.mAtGoalStatePub.publish(atGoalStateMsg)            

def main(args=None):
    rclpy.init(args=args)
    
    localization = Localization()
    print('Spinning Node...')
    rclpy.spin(localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    localization.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

