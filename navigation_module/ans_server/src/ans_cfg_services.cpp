#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_map_server/map_io.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("ans_cfg_server") {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting ANS server");
    declare_parameter("occ_map", "occ_map.yaml");
    get_parameter("occ_map", occ_map);
    declare_parameter("nav_map", "nav_map.pcl");
    get_parameter("nav_map", nav_map);            
    declare_parameter("goal_state");
    get_parameter("goal_state", goal_state);


    //Create Map Message Publisher
    //auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();
    mMapCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ans_services/map_pt_msg", qos);
    nav_timer_ = this->create_wall_timer(1s, std::bind(&ServerNode::load_nav_map, this));

    //Create Occupancy Map Message Publisher
    mMapOccupyPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/ans_services/occupancy_map_msg", qos);
    //occ_timer_ = this->create_wall_timer(1s, std::bind(&ServerNode::load_occupancy_map, this));
    
    //Create Goal State Message Publisher
    mGoalStatePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ans_services/goal_state_msg", qos);
    //goal_timer_ = this->create_wall_timer(1s, std::bind(&ServerNode::load_goal_state, this));
    
    


  }

private:

  void load_nav_map()
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Nav Map");
    //Initialize variables to store PCL transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 pcl_pc;
            
    //Read PCD file
    pcl::io::loadPCDFile<pcl::PointXYZ>(nav_map, *mapCloud);
    pcl::toPCLPointCloud2(*mapCloud, pcl_pc);
  
    //Generate Point Cloud Message
    sensor_msgs::msg::PointCloud2 mapMessage;
    pcl::toROSMsg(*mapCloud, mapMessage);
    mapMessage.header.frame_id = "/odom";

    // Create Publish Navigation Map
    mMapCloudPub->publish(mapMessage);  
  }
  
  void load_occupancy_map()
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Occupancy Map from %s", occ_map.c_str());
  
    //Convert input file to Occupancy grid
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    nav2_map_server::loadMapFromYaml(occ_map.c_str(), occupancy_grid_msg);

    // Create Publish Occupancy Map
    mMapOccupyPub->publish(occupancy_grid_msg);  
  }
  
  void load_goal_state()
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Goal State");

    // Create PoseStamped Msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "/odom";
    pose_msg.pose.position.x = goal_state[0];
    pose_msg.pose.position.y = goal_state[1];
    pose_msg.pose.position.z = goal_state[2];
    pose_msg.pose.orientation.x = goal_state[3];
    pose_msg.pose.orientation.y = goal_state[4];
    pose_msg.pose.orientation.z = goal_state[5];
    pose_msg.pose.orientation.w = goal_state[6];

    // Create Publish Goal State
    mGoalStatePub->publish(pose_msg);  
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mMapCloudPub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapOccupyPub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mGoalStatePub;

  std::string occ_map;
  std::string nav_map;
  std::vector<double> goal_state;
  rclcpp::TimerBase::SharedPtr nav_timer_;
  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}

// End of Code
