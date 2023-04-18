#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <iostream>
#include <fstream>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <string>

using namespace std::chrono_literals;

class lidar_scan_acq : public rclcpp::Node
{
    public: 
        lidar_scan_acq() : Node("lidar_scan_acq"), saveData_(false)
        {
            // Notify users of start of node
             RCLCPP_INFO(this->get_logger(), "Starting up lidar_scan_acq node");
            
            // Set saveData_ flah
            this->declare_parameter<bool>("saveData", saveData_);
            saveData_ = this->get_parameter("saveData").as_bool();
            
            // Set QoS parameters
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();
            
            // Create LiDAR scan message subscriber
            mLidarSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/rplidar_ros/scan", qos, 
                std::bind(&lidar_scan_acq::lidar_ptCloud_pub, 
                this, std::placeholders::_1));
                
           // Create LiDAR point cloud publisher
           mLidarPtCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_scan_acq/lidar_pt_msg", qos);
        }

    private:
    
        void lidar_ptCloud_pub(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
        {

            // Convert scan to point cloud msg
            auto lidarScanMsg = sensor_msgs::msg::PointCloud2();
            projector.projectLaser(*_msg, lidarScanMsg, -1.0, laser_geometry::channel_option::Intensity);
            
            // Publish LiDAR scan point cloud
            mLidarPtCloudPub->publish(lidarScanMsg);  
            
            // Save LiDAR scan if flag is set
            if ( saveData_ )
            {
            	// Convert LiDAR scan to PCL point cloud
            	auto pclPtCloud = pcl::PCLPointCloud2();
            	pcl_conversions::toPCL(lidarScanMsg, pclPtCloud);
            	
            	// Create Identity rotations and zeros vector for file writting
            	Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
            	Eigen::Vector4f v = Eigen::Vector4f::Zero();
            	
            	// Specify LiDAR scan filename
            	std::stringstream ss;
            	ss << "lidar_ptcloud_" << _msg->header.stamp.sec << "." << _msg->header.stamp.nanosec << ".pcd";
            	
            	// Write PCD file
            	pcl::PCDWriter writer;
            	const pcl::PCLPointCloud2& pclPtCloud_const = pclPtCloud;
            	writer.writeASCII(ss.str(), pclPtCloud_const, v, q, 8);          	
            }
  	}
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mLidarSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPtCloudPub;
        laser_geometry::LaserProjection projector;
        std::atomic<bool> saveData_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<lidar_scan_acq>());
	rclcpp::shutdown();
	return 0;
}
