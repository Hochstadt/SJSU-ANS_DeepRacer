#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_map_server/map_io.hpp>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <string>
#include <ans_msgs/srv/file_path.hpp>

#include <memory>

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("ans_cfg_server") {

    //Create Map Loader Service
    nav_map_srv = create_service<ans_msgs::srv::FilePath>(
        "nav_map_loader", std::bind(&ServerNode::load_nav_map, this,
         std::placeholders::_1, std::placeholders::_2));
         
    //Create Occupancy Map Loader Service
    occ_map_srv = create_service<ans_msgs::srv::FilePath>(
        "occupancy_map_loader", std::bind(&ServerNode::load_occupancy_map, this,
         std::placeholders::_1, std::placeholders::_2));                                
         
    //Create Goal State Loader Service
    gs_srv = create_service<ans_msgs::srv::FilePath>(
        "goal_state_loader", std::bind(&ServerNode::load_goal_state, this,
         std::placeholders::_1, std::placeholders::_2));      
                                                                   
    //Create Map Message Publisher
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    mMapCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ans_services/map_pt_msg", qos);
    
    //Create Occupancy Map Message Publisher
    mMapOccupyPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/ans_services/occupancy_map_msg", qos);
    
    //Create Goal State Message Publisher
    mGoalStatePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ans_services/goal_state_msg", qos);
    
    // Message users of status
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Prepared to load files");
  }

private:

  void load_nav_map(std::shared_ptr<ans_msgs::srv::FilePath::Request> request,
	            std::shared_ptr<ans_msgs::srv::FilePath::Response> response)
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Navigation Map from %s", request->file_path.c_str());
  
    //Initialize variables to store PCL transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc;
            
    //Read PCD file
    pcl::io::loadPCDFile<pcl::PointXYZ>(request->file_path, *mapCloud);
    pcl::toPCLPointCloud2(*mapCloud, pcl_pc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data points in cloud: %d", mapCloud->width*mapCloud->height);
  
    //Generate Point Cloud Message
    sensor_msgs::msg::PointCloud2 mapMessage;
    pcl::toROSMsg(*mapCloud, mapMessage);
    mapMessage.header.frame_id = "/odom";

    // Create Publish Navigation Map
    mMapCloudPub->publish(mapMessage);  
    response->success = true;
  }
  
  void load_occupancy_map(std::shared_ptr<ans_msgs::srv::FilePath::Request> request,
	                  std::shared_ptr<ans_msgs::srv::FilePath::Response> response)
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Occupancy Map from %s", request->file_path.c_str());
  
    //Convert input file to Occupancy grid
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    nav2_map_server::loadMapFromYaml(request->file_path, occupancy_grid_msg);

    // Create Publish Occupancy Map
    mMapOccupyPub->publish(occupancy_grid_msg);  
    response->success = true;
  }
  
  void load_goal_state(std::shared_ptr<ans_msgs::srv::FilePath::Request> request,
	               std::shared_ptr<ans_msgs::srv::FilePath::Response> response)
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Goal State from %s", request->file_path.c_str());
  
    //Initialize variables to store PCL transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc;
            
    //Read PCD file
    pcl::io::loadPCDFile<pcl::PointXYZ>(request->file_path, *mapCloud);
    pcl::toPCLPointCloud2(*mapCloud, pcl_pc);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Data points in cloud: %d", mapCloud->width*mapCloud->height);
  
    //Generate Point Cloud Message
    sensor_msgs::msg::PointCloud2 mapMessage;
    pcl::toROSMsg(*mapCloud, mapMessage);
    mapMessage.header.frame_id = "/odom";

    // Create Publish Goal State
    //mGoalStatePub->publish(mapMessage);  
    response->success = true;
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapCloudPub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapOccupyPub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalStatePub;
  rclcpp::Service<ans_msgs::srv::FilePath>::SharedPtr nav_map_srv;
  rclcpp::Service<ans_msgs::srv::FilePath>::SharedPtr occ_map_srv;
  rclcpp::Service<ans_msgs::srv::FilePath>::SharedPtr gs_srv;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}

// End of Code
