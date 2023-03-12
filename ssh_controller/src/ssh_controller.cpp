#include "rclcpp/rclcpp.hpp"
#include "deepracer_interfaces_pkg/srv/video_state_srv.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //node generally is shared ptr
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("activate_camera_client");
  //create the client
  rclcpp::Client<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr activateCameraClient = 
    node->create_client<deepracer_interfaces_pkg::srv::VideoStateSrv>("/camera_pkg/media_state");
  //get the request that you will be making
  auto request = std::make_shared<deepracer_interfaces_pkg::srv::VideoStateSrv::Request>();
  //we want to enable this camera streaming to start with, unsure how we'll stop it but ctrl+C works!
  request->activate_video = 1; 

  //now we wait...
  while (!activateCameraClient->wait_for_service(1s))
  {
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "INterrupted while waiting for service.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available waiting again...");
  }
  //send the request
  auto result = activateCameraClient->async_send_request(request);

  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call the media_state service");
  }

  rclcpp::shutdown();
  return 0;


}
