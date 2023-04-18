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
#include <pcl/registration/gicp.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
                "/map_loader/map_pt_msg", 10, 
                std::bind(&localizer::store_map, 
                this, std::placeholders::_1));
                
            // Create LiDAR scan message subscriber
            mLidarPtCloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/lidar_scan_acq/lidar_pt_msg", qos, 
                std::bind(&localizer::estimate_pose, 
                this, std::placeholders::_1));
                
           // Create Goal State message subscriber
            mGoalStateSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_state/pose", qos, 
                std::bind(&localizer::store_goal_state, 
                this, std::placeholders::_1));
                
           // Create LiDAR scan message subscriber
           mPoseEstPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localizer/pose", qos);
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
                   
            // Inform that message was received
            RCLCPP_INFO(this->get_logger(), "LiDAR Point Cloud has been received  with %i points",_msg->height*_msg->width);
            
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
            icp.align(unused);
            
            // Get Pose Estimate
            const Eigen::Matrix4f T = icp.getFinalTransformation();
            std::cout << "Pose: " << T << std::endl;
            
            // Check Goal State
            //at_goal_state(T);
            
            // Convert Pose to PoseStamped msg
            geometry_msgs::msg::PoseStamped ros_pose;
            ros_pose = localizer::convert_pose_to_msg(T);
            
            // Publish Pose
            ros_pose.header.frame_id = "/odom";
            mPoseEstPub->publish(ros_pose);                          
           
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
            // Compare poses
            // do something here
            
            // set goal state flag
            atGoalState = true;  
        }      
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mMapSub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPtCloudSub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalStateSub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPoseEstPub;
	pcl::PCLPointCloud2 envMapPCL2;
	tf2::Transform goalState;
	bool localizationType;
	bool solutionFound;
	bool atGoalState;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<localizer>());
	rclcpp::shutdown();
	return 0;
}
