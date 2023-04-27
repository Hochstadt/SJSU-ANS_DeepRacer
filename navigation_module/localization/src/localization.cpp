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
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen/tf2_eigen.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();
            
            // Create Map Load message subscriber
            mMapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ans_services/map_pt_msg", 10, 
                std::bind(&localization::store_map, 
                this, std::placeholders::_1));
            //create bool for it too
            receivedMap = false;
                
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
        }

    private:
    
        void store_map(const sensor_msgs::msg::PointCloud2::SharedPtr _msg)
        {
                   
            // Inform that message was received
            RCLCPP_INFO(this->get_logger(), "Map has been received  with %i points",_msg->height*_msg->width);
            
            // Convert LiDAR scan to PCL point cloud
            pcl_conversions::toPCL(*_msg, envMapPCL2);
            receivedMap = true;
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
            //Check if we received the map before doign this...
            if(receivedMap == true)
            {            

              RCLCPP_INFO(this->get_logger(), "Initializing ICP");
              // Initialize variables to store PCL transformation
              pcl::PCLPointCloud2 tmpLidar;
              pcl::PointCloud<pcl::PointXYZ>::Ptr LidarPtCloud(new pcl::PointCloud<pcl::PointXYZ>);
              pcl::PointCloud<pcl::PointXYZ>::Ptr envMap(new pcl::PointCloud<pcl::PointXYZ>); 
              
              // Convert LiDAR scan to PCL point cloud
              pcl_conversions::toPCL(*_msg, tmpLidar);
              pcl::fromPCLPointCloud2(tmpLidar, *LidarPtCloud);    
              
              // Convert Environment Map to PCL point cloud
              pcl::fromPCLPointCloud2(envMapPCL2, *envMap);            
              
              // Perform ICP
              GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
              icp.setInputSource(LidarPtCloud);
              icp.setInputTarget(envMap);
              pcl::PointCloud<pcl::PointXYZ> unused;
              RCLCPP_INFO(this->get_logger(), "Running ICP");
              icp.align(unused);
              if(icp.hasConverged() == true){
                RCLCPP_INFO(this->get_logger(), "ICP Converged");
              } else {
                RCLCPP_INFO(this->get_logger(), "ICP NOT Converged");
              }
              // Get Pose Estimate
              const Eigen::Matrix4f T = icp.getFinalTransformation();
              
                          
              // Check Goal State
              at_goal_state(T);
              
              // Convert Pose to PoseStamped msg
              geometry_msgs::msg::PoseStamped ros_pose;
              ros_pose = localization::convert_pose_to_msg(T);
              
              // Publish Pose
              ros_pose.header.frame_id = "/odom";
              ros_pose.header.stamp.sec = _msg->header.stamp.sec;
              ros_pose.header.stamp.nanosec = _msg->header.stamp.nanosec;
              RCLCPP_INFO(this->get_logger(), "Returning Result");
              mPoseEstPub->publish(ros_pose);                          
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
            if (abs(Tgoal_current(0,1)) < rotationThreshold && sqrt(pow(Tgoal_current(0,3), 2) + pow(Tgoal_current(1,3), 2) + pow(Tgoal_current(2,3), 2)) < positionThrehsold) {
            	// set goal state flag
            	atGoalStateMsg.data = true;  
            	
            } else {
            	// set goal state flag
            	atGoalStateMsg.data = false;
            	
            }
            
            // Publish goal state message
            mAtGoalStatePub->publish(atGoalStateMsg);
        }      
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mMapSub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPtCloudSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalStateSub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseEstPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mAtGoalStatePub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mSolutionFoundPub;
	pcl::PCLPointCloud2 envMapPCL2;
	Eigen::Affine3d goalState;
	bool localizationType;
	bool solutionFound;
	bool atGoalState;
        bool receivedMap;
	double rotationThreshold = 0.017;
	double positionThrehsold = 0.010;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<localization>());
	rclcpp::shutdown();
	return 0;
}
