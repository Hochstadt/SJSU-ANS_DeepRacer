#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <tf2/LinearMath/Matrix3x3.h>

//#include <tf2_eigen/tf2_eigen.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/transforms.h>
#include "ros_user_macros.h"

using pcl::IterativeClosestPoint;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::transformPointCloud;

using namespace std::chrono_literals;

class localization : public rclcpp::Node
{
    public: 
        localization() : Node("localization")
        {
            // Notify users of start of node
            RCLCPP_INFO(this->get_logger(), "Starting localization node");
            
            // Set QoS parameters
            auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
            qos.best_effort();
                       
            // Create Map Load message subscriber
            mMapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ans_services/map_pt_msg", 10, 
                std::bind(&localization::store_map, 
                this, std::placeholders::_1));
                
            // Create LiDAR scan message subscriber
            mLidarPtCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/lidar_scan_acq/lidar_pt_msg", qos, 
                std::bind(&localization::estimate_pose, 
                this, std::placeholders::_1));
                
            // Create Goal State message subscriber
            mGoalStateSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/ans_services/goal_state_msg", qos, 
                std::bind(&localization::store_goal_state, 
                this, std::placeholders::_1));

            // Create Localization Pose Estimate publisher
            mPoseEstPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", qos);
           
            // Create atGoalState publisher
            mAtGoalStatePub = this->create_publisher<std_msgs::msg::Bool>("/localization/at_goal_state_msg", qos);
           
            // Create solutionFound publisher
            mSolutionFoundPub = this->create_publisher<std_msgs::msg::Bool>("/localization/solution_found", qos);
           
            // Create LiDAR point cloud publisher
            mAlignedLidarPtCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/localization/aligned_lidar_pt_msg", qos);
        
            // Declare input variables
            #if ROS_USER_VERSION == ROS_HUMBLE
                this->declare_parameter<float>("icp_fit_thresh");
                this->declare_parameter<float>("gicp_fit_thresh");
                this->declare_parameter<float>("icp_trans_eps");
                this->declare_parameter<float>("icp_max_corr_dist");
                this->declare_parameter<int>("icp_max_iters");
                this->declare_parameter<float>("min_search_x");
                this->declare_parameter<float>("max_search_x");
                this->declare_parameter<int>("delta_search_x");
                this->declare_parameter<float>("min_search_y");
                this->declare_parameter<float>("max_search_y");
                this->declare_parameter<int>("delta_search_y");
                this->declare_parameter<int>("delta_search_ang");
                this->declare_parameter<float>("rotationCheckThreshold");
                this->declare_parameter<float>("positionCheckThrehsold");

                // Declare set input variables
                icp_fit_thresh = this->get_parameter("icp_fit_thresh").as_double();
                gicp_fit_thresh = this->get_parameter("gicp_fit_thresh").as_double();
                min_search_x = this->get_parameter("min_search_x").as_double();
                max_search_x = this->get_parameter("max_search_x").as_double();
                delta_search_x = this->get_parameter("delta_search_x").as_int();
                min_search_y = this->get_parameter("min_search_y").as_double();
                max_search_y = this->get_parameter("max_search_y").as_double();
                delta_search_y = this->get_parameter("delta_search_y").as_int();
                delta_search_ang = this->get_parameter("delta_search_ang").as_int();
                rotationCheckThreshold = this->get_parameter("rotationCheckThreshold").as_double();
                positionCheckThrehsold = this->get_parameter("positionCheckThrehsold").as_double();

                // Configure ICP and GICP
                icp.setTransformationEpsilon(this->get_parameter("icp_trans_eps").as_double());
                icp.setMaxCorrespondenceDistance(this->get_parameter("icp_max_corr_dist").as_double());
                icp.setMaximumIterations(this->get_parameter("icp_max_iters").as_int());
                gicp.setTransformationEpsilon(this->get_parameter("icp_trans_eps").as_double());
                gicp.setMaxCorrespondenceDistance(this->get_parameter("icp_max_corr_dist").as_double());
                gicp.setMaximumIterations(this->get_parameter("icp_max_iters").as_int());
            #elif ROS_USER_VERSION == ROS_FOXY
                declare_parameter("icp_fit_thresh");
                declare_parameter("gicp_fit_thresh");
                declare_parameter("icp_trans_eps");
                declare_parameter("icp_max_corr_dist");
                declare_parameter("icp_max_iters");
                declare_parameter("min_search_x");
                declare_parameter("max_search_x");
                declare_parameter("delta_search_x");
                declare_parameter("min_search_y");
                declare_parameter("max_search_y");
                declare_parameter("delta_search_y");
                declare_parameter("delta_search_ang");
                declare_parameter("rotationCheckThreshold");
                declare_parameter("positionCheckThrehsold");

                // Declare set input variables
                icp_fit_thresh = this->get_parameter("icp_fit_thresh").as_double();
                gicp_fit_thresh = this->get_parameter("gicp_fit_thresh").as_double();
                min_search_x = this->get_parameter("min_search_x").as_double();
                max_search_x = this->get_parameter("max_search_x").as_double();
                delta_search_x = this->get_parameter("delta_search_x").as_int();
                min_search_y = this->get_parameter("min_search_y").as_double();
                max_search_y = this->get_parameter("max_search_y").as_double();
                delta_search_y = this->get_parameter("delta_search_y").as_int();
                delta_search_ang = this->get_parameter("delta_search_ang").as_int();
                rotationCheckThreshold = this->get_parameter("rotationCheckThreshold").as_double();
                positionCheckThrehsold = this->get_parameter("positionCheckThrehsold").as_double();

                // Configure ICP and GICP
                icp.setTransformationEpsilon(get_parameter("icp_trans_eps").as_double());
                icp.setMaxCorrespondenceDistance(get_parameter("icp_max_corr_dist").as_double());
                icp.setMaximumIterations(get_parameter("icp_max_iters").as_int());
                gicp.setTransformationEpsilon(get_parameter("icp_trans_eps").as_double());
                gicp.setMaxCorrespondenceDistance(get_parameter("icp_max_corr_dist").as_double());
                gicp.setMaximumIterations(get_parameter("icp_max_iters").as_int());
            #endif

        }

    private:
    
        void store_map(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
        {
                   
            // Inform that message was received
            RCLCPP_INFO(this->get_logger(), "Map has been received  with %i points",_msg->height*_msg->width);
            
            // Convert LiDAR scan to PCL point cloud
            pcl_conversions::toPCL(*_msg, envMapPCL2);

  	    }
  	
  	    void store_goal_state(const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
        {
                   
            // Inform that message was received
            RCLCPP_INFO(this->get_logger(), "Goal State received");
            
            // Convert LiDAR scan to PCL point cloud
            tf2::fromMsg(_msg->pose, goalState);
  	    }
        
        void estimate_pose(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
        {
            // Verify map is loaded
            if (envMapPCL2.width > 0) {
                RCLCPP_INFO(this->get_logger(), "Initializing ICP");
                // Initialize variables to store PCL objects
                pcl::PCLPointCloud2 tmpLidar;
                pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScan(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScanTransformed(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr envMap(new pcl::PointCloud<pcl::PointXYZ>); 
                pcl::PointCloud<pcl::PointXYZ> aligned;
                
                // Convert LiDAR scan to PCL point cloud
                //pcl_conversions::toPCL(*_msg, tmpLidar);
                pcl::fromROSMsg(*_msg, *lidarScan);    
                
                // Convert Environment Map to PCL point cloud
                pcl::fromPCLPointCloud2(envMapPCL2, *envMap); 

                // Perform pose search if no a priori knowledge
                if(solutionFound==false) {
                    RCLCPP_INFO(this->get_logger(), "Running Global ICP as no solution has been found");
                    currentState = searchForInitialPose(lidarScan, lidarScanTransformed, aligned, envMap);
                } else {
                    //Perform ICP with previous pose infromation
                    RCLCPP_INFO(this->get_logger(), "Running Local ICP as we have a solution");
                    // Transform point cloud based on a priori pose
                    pcl::transformPointCloud(*lidarScan, *lidarScanTransformed, currentState); 
                    
                    // Attempt align LiDAR scan and map
                    gicp.setInputSource(lidarScanTransformed);
                    gicp.setInputTarget(envMap);
                    gicp.align(aligned);

                    // Store and confirm pose if quality solution produced
                    if (gicp.getFitnessScore() < gicp_fit_thresh) 
                    {
                        // Get Pose Estimate
                        currentState = gicp.getFinalTransformation()*currentState;
                        solutionFound = true;
                    }
                    // Indicate no knowledge if low quality solution produced
                    else
                    {
                        solutionFound = false;  
                        currentState = Eigen::Matrix4f::Identity();
                    }
                }

                // Produce aligned point cloud mesage
                auto AlignedMsg = sensor_msgs::msg::PointCloud2();
                pcl::toROSMsg(aligned, AlignedMsg);
                AlignedMsg.header.frame_id ="odom";
                mAlignedLidarPtCloudPub->publish(AlignedMsg);
                            
                // Check Goal State
                at_goal_state(currentState);
                
                // Convert Pose to PoseStamped msg
                geometry_msgs::msg::PoseStamped ros_pose;
                ros_pose = localization::convert_pose_to_msg(currentState);
                
                // Publish Pose
                ros_pose.header.frame_id = "odom";
                ros_pose.header.stamp.sec = _msg->header.stamp.sec;
                ros_pose.header.stamp.nanosec = _msg->header.stamp.nanosec;
                mPoseEstPub->publish(ros_pose);

                // Publish Solution State
                std_msgs::msg::Bool solutionFoundMsg;
                solutionFoundMsg.data = solutionFound;
                mSolutionFoundPub->publish(solutionFoundMsg);              
            
            }
        }
  	
  	    inline geometry_msgs::msg::PoseStamped convert_pose_to_msg(Eigen::Matrix4f T)
        {
                   
            // Convert DCM to Angles
            double roll, pitch, yaw;
            tf2::Matrix3x3 dcm(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
            dcm.getRPY(roll, pitch, yaw);
            
            // Convert Angles to Quaterion
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            
            // Create PoseStamped Msg
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.pose.position.x = T(0, 3);
            pose_msg.pose.position.y = T(1, 3);
            pose_msg.pose.position.z = T(2, 3);
            pose_msg.pose.orientation.x = quat.getX();
            pose_msg.pose.orientation.y = quat.getY();
            pose_msg.pose.orientation.z = quat.getZ();
            pose_msg.pose.orientation.w = quat.getW();
            
            return pose_msg;
  	    }
  	
  	    void at_goal_state(Eigen::Matrix4f pose)
  	    {
            // Transform goal state to Eigen Matrix
            Eigen::Matrix4d Tgoal_state;
            Tgoal_state = goalState.matrix();
            
            // Get Transform from Current State and Goal State
            Eigen::Matrix4d Tgoal_current = Tgoal_state.inverse() * pose.cast<double>();
            
            // Initial message
            std_msgs::msg::Bool atGoalStateMsg;
            
            // Check if current state within threshold of goal
            if (abs(Tgoal_current(0,1)) < rotationCheckThreshold && sqrt(pow(Tgoal_current(0,3), 2) + pow(Tgoal_current(1,3), 2) + pow(Tgoal_current(2,3), 2)) < positionCheckThrehsold) {
            	// set goal state flag
            	atGoalStateMsg.data = true;  
            	
            } else {
            	// set goal state flag
            	atGoalStateMsg.data = false;
            	
            }
            
            // Publish goal state message
            mAtGoalStatePub->publish(atGoalStateMsg);
        }      
        
        Eigen::Matrix4f searchForInitialPose(pcl::PointCloud<pcl::PointXYZ>::Ptr scan, pcl::PointCloud<pcl::PointXYZ>::Ptr scanTransformed, pcl::PointCloud<pcl::PointXYZ> & aligned, pcl::PointCloud<pcl::PointXYZ>::Ptr map)
        {
       
            // Initialize variables
            float fitScore = std::numeric_limits<float>::max();
            Eigen::Matrix4f pose;

            // Loop through each position and angle combination
       	    for (int x=0; x < delta_search_x; x++){
                for (int y=0; y < delta_search_y; y++){
                    for (int theta=0; theta < delta_search_ang; theta++){
            		
                        // Define pose conditions
            		    float pos_x = min_search_x + x * ((max_search_x - min_search_x) / delta_search_x);
            		    float pos_y = min_search_y + y * ((max_search_y - min_search_y) / delta_search_y);
            		    float ang   = theta * (3.14159265358979323846 / delta_search_ang);

                        // Construct initial pose guess
            		    Eigen::Matrix4f initialEst = Eigen::Matrix4f::Identity();
            		    initialEst(0,0) = cos(ang);
            		    initialEst(0,1) = -sin(ang);
            		    initialEst(1,0) = sin(ang);
            		    initialEst(1,1) = cos(ang);
            		    initialEst(0,3) = pos_x;
            		    initialEst(1,3) = pos_y;
            				
            	        // Transform point cloud based on a priori pose
            		    pcl::transformPointCloud(*scan, *scanTransformed, initialEst);
            			
            		    // Attempt alignment with ICP	
            		    icp.setInputSource(scanTransformed);
            		    icp.setInputTarget(map);
             		    icp.align(aligned);
            			
            		    // Save pose estimate if lowest fitness seen
            		    float FitnessScore = icp.getFitnessScore();
                        if (FitnessScore < fitScore)
            		    {
            			    // Get Pose Estimate
            			    pose = icp.getFinalTransformation()*initialEst;
            			    fitScore = FitnessScore;
				        }
			        }
		        }
	        }

            // Save pose if fitness below threshold
            RCLCPP_INFO(this->get_logger(), "Fitness Score: %.4f",fitScore);
            RCLCPP_INFO(this->get_logger(), "Fitness Threshold: %.4f",icp_fit_thresh);
            if (fitScore < icp_fit_thresh){
                solutionFound = true;
                return pose;
            }
	
            // Return nothing if no good solution found
            solutionFound = false;
	        return Eigen::Matrix4f::Identity();
	    }
        
        // Define Subscription variables
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mMapSub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPtCloudSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalStateSub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseEstPub;

        // Define Publisher variables
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mAtGoalStatePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mSolutionFoundPub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mAlignedLidarPtCloudPub;

        // Define State Varaibles
	    pcl::PCLPointCloud2 envMapPCL2;
	    Eigen::Affine3d goalState;
	    bool solutionFound = false;
	    bool atGoalState = false;
        Eigen::Matrix4f currentState = Eigen::Matrix4f::Identity();

        // Define ICP and GICP variables
        IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        float icp_trans_eps;
        float icp_max_corr_dist;
        int icp_max_iters;

        // Define configurable parameters
	    float rotationCheckThreshold;
	    float positionCheckThrehsold;
        float icp_fit_thresh;
        float gicp_fit_thresh;
        int min_search_x;
        float max_search_x;
        int delta_search_x;
        float min_search_y;
        float max_search_y;
        int delta_search_y;
        int delta_search_ang;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<localization>());
	rclcpp::shutdown();
	return 0;
}
