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


  rclcpp::Client<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr activateDataCollectClient = 
    node->create_client<deepracer_interfaces_pkg::srv::VideoStateSrv>("/activate_data_collection");
  //also want to make the same request of the data_collector. To not have to make our own type
  //I just used the same .srv of the VideoStateSrv
  auto dc_request = std::make_shared<deepracer_interfaces_pkg::srv::VideoStateSrv::Request>();
  dc_request->activate_video = 1; 
  

  //repeat for data collection
  while (!activateDataCollectClient->wait_for_service(1s))
  {
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for service.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available waiting again...");
  }
  //send the request
  auto dc_result = activateDataCollectClient->async_send_request(dc_request);
  if(rclcpp::spin_until_future_complete(node, dc_result) == rclcpp::FutureReturnCode::TIMEOUT)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call the activate_data_collection service");
  }

  rclcpp::shutdown();
  return 0;


}
