#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <iostream>
#include <fstream>
#include <laser_geometry/laser_geometry.hpp>
#include <string>

using namespace std::chrono_literals;

class lidar_scan_acq : public rclcpp::Node
{
    public: 
        lidar_scan_acq() : Node("lidar_scan_acq")
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            qos.best_effort();
            
            std::cout << "mSubscription " << std::endl;
            mLidarSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/rplidar_ros/scan", qos, 
                std::bind(&lidar_scan_acq::lidar_ptCloud_pub, 
                this, std::placeholders::_1));
                
           mLidarPtCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_scan_acq/lidar_pt_msg", qos);
        }

    private:
    
        void lidar_ptCloud_pub(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
        {
                   
            // Inform that message was received
            RCLCPP_INFO(this->get_logger(), "TimeStamp: '%f' - Number Meas: %u - Min Range: %f - Max Range: %f", _msg->header.stamp.sec + _msg->header.stamp.nanosec/1e9, _msg->ranges.size(), _msg->range_min, _msg->range_max);

            // Convert scan to point cloud msg
            auto lidarScanMsg = sensor_msgs::msg::PointCloud2();
            projector.projectLaser(*_msg, lidarScanMsg, -1.0, laser_geometry::channel_option::Intensity);
            
            //std::cout << "timestamp: " << _msg->header.stamp.sec << "." << _msg->header.stamp.nanosec << std::endl;
            //std::cout << "timestamp: " << lidarScanMsg.header.stamp.sec << "." << lidarScanMsg.header.stamp.nanosec << std::endl;
            
            // Publish LiDAR scan point cloud
            mLidarPtCloudPub->publish(lidarScanMsg);            

  	}
        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mLidarSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mLidarPtCloudPub;
        laser_geometry::LaserProjection projector;        
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<lidar_scan_acq>());
	rclcpp::shutdown();
	return 0;
}
