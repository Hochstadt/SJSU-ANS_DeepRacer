#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_map_server/map_io.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
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

    //Create Map Message Publisher
    //auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
    qos.best_effort();

    //Create Occupancy Map Message Publisher
    mMapOccupyPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/ans_services/occupancy_map_msg", qos);
    occ_timer_ = this->create_wall_timer(1s, std::bind(&ServerNode::load_occupancy_map, this));

  }

private:

  void load_occupancy_map()
  {
    // Notify users of start of node
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Occupancy Map from %s", occ_map.c_str());
  
    //Convert input file to Occupancy grid
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    nav2_map_server::loadMapFromYaml(occ_map.c_str(), occupancy_grid_msg);
    occupancy_grid_msg.header.frame_id = "odom";

    // Create Publish Occupancy Map
    mMapOccupyPub->publish(occupancy_grid_msg);  
  }
  
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mMapOccupyPub;

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
