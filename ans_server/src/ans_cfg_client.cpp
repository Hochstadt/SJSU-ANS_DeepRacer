#include "rclcpp/rclcpp.hpp"
#include "ans_msgs/srv/file_path.hpp"
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
  
  std::cout << service_call << std::endl;
  
  //Set Client
  rclcpp::Client<ans_msgs::srv::FilePath>::SharedPtr client;
  if (service_call == "load_nav_map"){
      client = node->create_client<ans_msgs::srv::FilePath>("nav_map_loader");
      
  } else if (service_call == "load_occupancy_map"){
      client = node->create_client<ans_msgs::srv::FilePath>("occupancy_map_loader");
      
  } else if (service_call == "load_goal_state"){
      client = node->create_client<ans_msgs::srv::FilePath>("goal_state_loader");
      
  } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incorrect Service Name");
      return 0;
  }

  //Generate request for service
  auto request = std::make_shared<ans_msgs::srv::FilePath::Request>();
  request->file_path = argv[2];

  //Check if service is responding
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  // Wait for the result
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "File Successfully Loaded");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load the file");
  }

  rclcpp::shutdown();
  return 0;
}
