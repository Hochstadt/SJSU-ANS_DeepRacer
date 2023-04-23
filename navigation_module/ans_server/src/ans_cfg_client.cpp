#include "rclcpp/rclcpp.hpp"
#include "ans_msgs/srv/file_path.hpp"
#include <ans_msgs/srv/pose.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  //Set server type
  std::string service_call = argv[1];
  
  //Create Client Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ans_cfg_client");
  
  //Set Client
  if (service_call == "load_nav_map"){
      rclcpp::Client<ans_msgs::srv::FilePath>::SharedPtr client;
      client = node->create_client<ans_msgs::srv::FilePath>("nav_map_loader");
      
      //Generate request for service
      auto request = std::make_shared<ans_msgs::srv::FilePath::Request>();
      request->file_path = argv[2];
      
      // Wait for the result
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input Successfully Loaded");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Input");
      }
      
  } else if (service_call == "load_occupancy_map"){
      rclcpp::Client<ans_msgs::srv::FilePath>::SharedPtr client;
      client = node->create_client<ans_msgs::srv::FilePath>("occupancy_map_loader");
      
      //Generate request for service
      auto request = std::make_shared<ans_msgs::srv::FilePath::Request>();
      request->file_path = argv[2];

      // Wait for the result
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input Successfully Loaded");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Input");
      }
      
  } else if (service_call == "load_goal_state"){
      rclcpp::Client<ans_msgs::srv::Pose>::SharedPtr client;
      client = node->create_client<ans_msgs::srv::Pose>("goal_state_loader");
      
      //Generate request for service
      auto request = std::make_shared<ans_msgs::srv::Pose::Request>();
      request->pos_x  = std::stod(argv[2]);
      request->pos_y  = std::stod(argv[3]);
      request->pos_z  = std::stod(argv[4]);
      request->quat_x = std::stod(argv[5]);
      request->quat_y = std::stod(argv[6]);
      request->quat_z = std::stod(argv[7]);
      request->quat_w = std::stod(argv[8]);

      // Wait for the result
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Input Successfully Loaded");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Input");
      }
      
  } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incorrect Service Name");
      return 0;
  }


  rclcpp::shutdown();
  return 0;
}
