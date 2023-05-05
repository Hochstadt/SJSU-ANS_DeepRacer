#include "rclcpp/rclcpp.hpp"
#include "ans_msgs/srv/file_path.hpp"
#include <ans_msgs/srv/pose.hpp>
#include "ans_msgs/srv/start_flag.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
using namespace std::chrono_literals;


class ans_cfg_client : public rclcpp::Node
{
    public: 
        ans_cfg_client() : Node("ans_cfg_client")
        {

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting ANS Client");
            // Set the incoming parameters
            declare_parameter("occ_map", "occ_map.yaml");
            get_parameter("occ_map", occ_map);
            declare_parameter("nav_map", "nav_map.pcl");
            get_parameter("nav_map", nav_map);            
            declare_parameter("goal_state");
            get_parameter("goal_state", goal_state);




            //Create Goal State Loader Service
            start_srv = create_service<ans_msgs::srv::StartFlag>(
              "start_loading", std::bind(&ans_cfg_client::start_loading, this,
              std::placeholders::_1, std::placeholders::_2));  
        }


    private:
      void start_loading(std::shared_ptr<ans_msgs::srv::StartFlag::Request> request,
	        std::shared_ptr<ans_msgs::srv::StartFlag::Response> response)
      {

        if(request->start == true)
        {
          ////////////////////////////////////////////
          //send the nav map
          ////////////////////////////////////////////
          rclcpp::Client<ans_msgs::srv::FilePath>::SharedPtr fclient;
          fclient = this->create_client<ans_msgs::srv::FilePath>("nav_map_loader");
          
          //Generate request for service
          auto nav_request = std::make_shared<ans_msgs::srv::FilePath::Request>();
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading Nav map: %s", nav_map.c_str());
          nav_request->file_path = nav_map;
          
          // Wait for the result
          auto nav_result = fclient->async_send_request(nav_request);
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), nav_result) == rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Nav Map Successfully Loaded");
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Nav Map");
          }
          
          ////////////////////////////////////////////
          //send the occupancy map
          ////////////////////////////////////////////            
          fclient = this->create_client<ans_msgs::srv::FilePath>("occupancy_map_loader");
          
          //Generate request for service
          auto occ_request = std::make_shared<ans_msgs::srv::FilePath::Request>();
          occ_request->file_path = occ_map;

          // Wait for the result
          auto occ_result = fclient->async_send_request(occ_request);
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), occ_result) == rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Occ Map Successfully Loaded");
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Occ Map");
          }

          ////////////////////////////////////////////
          //send the goal state
          ////////////////////////////////////////////  
          rclcpp::Client<ans_msgs::srv::Pose>::SharedPtr pclient;  
          pclient = this->create_client<ans_msgs::srv::Pose>("goal_state_loader");
          
          //Generate request for service
          auto goal_request = std::make_shared<ans_msgs::srv::Pose::Request>();
          goal_request->pos_x  = goal_state[0];
          goal_request->pos_y  = goal_state[1];
          goal_request->pos_z  = goal_state[2];
          goal_request->quat_x = goal_state[3];
          goal_request->quat_y = goal_state[4];
          goal_request->quat_z = goal_state[5];
          goal_request->quat_w = goal_state[6];

          // Wait for the result
          auto goal_result = pclient->async_send_request(goal_request);
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_result) == rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully Loaded Goal State");
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to load Goal State");
          }
          response->success = true;
        } else {
          response->success = false;
        }
      }

    

    rclcpp::Service<ans_msgs::srv::StartFlag>::SharedPtr start_srv;
    std::string occ_map;
    std::string nav_map;
    std::vector<double> goal_state;
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ans_cfg_client>());
  rclcpp::shutdown();
  return 0;
}
  //string_map_path = 
  //string_occ_map_path =
  //double goal_state [7]; 
  

  //Set server type
  //std::string service_call = argv[1];
  
  //Create Client Node
  /*
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ans_cfg_client");
  

  // Set saveData_ flag
  std::string occ_map_name;
  node->declare_parameter<std::string>("occ_map_name", occ_map_name);
  occ_map_name = this->get_parameter("occ_map_name").as_string();

  //Set Client
  
  if (service_call == "load_nav_map"){
      rclcpp::Client<ans_msgs::srv::FilePath>::SharedPtr client;
      client = node->create_client<ans_msgs::srv::FilePath>("nav_map_loader");
      
      //Generate request for service
      auto request = std::make_shared<ans_msgs::srv::FilePath::Request>();
      request->file_path = argv[2];
      
      // Wait for the result
      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

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
      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

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
      while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

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
*/