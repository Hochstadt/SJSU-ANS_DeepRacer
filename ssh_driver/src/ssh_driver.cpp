#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "deepracer_interfaces_pkg/srv/servo_gpio_srv.hpp"
#include <chrono>
#include <cstdio>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SshDriver : public rclcpp::Node
{
    public:
        SshDriver() : Node("ssh_driver")
        {
            //subsciption = this->create_subscription<std_msgs::msg::String>(
            //    "cmd_vel", 10, std::bind(&SshDriver::acceptCmd, this, _1));
            mSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&SshDriver::acceptCmd, this, _1));
	    mServoGPIOClient = this->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>("/servo_pkg/servo_gpio");
            //setup quality of service profile (ref. deepracer)
            //keeplast - will keep single sample
            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            //besteffort - will try to deliver samples, but may lose them if network is bad 
            qos.best_effort();
                                              //<deepracer_interfaces_pkg::msg::ServoCtrlMsg>
	    mServoPub = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", qos);


        }

    private:
        void acceptCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Command Received");
            RCLCPP_INFO(this->get_logger(), "Linear: <%.2f, %.2f, %.2f>",
                msg->linear.x, msg->linear.y, msg->linear.z );
            RCLCPP_INFO(this->get_logger(), "Angular: <%.2f, %.2f, %.2f>", 
                msg->angular.x, msg->angular.y, msg->angular.z);
            
            if(0.0 == abs(msg->linear.x) + abs(msg->linear.y) + abs(msg->linear.z) + 
              abs(msg->angular.x) + abs(msg->angular.y) + abs(msg->angular.z)) {
              mThrottle = 0;
              bStop = true;
              RCLCPP_INFO(this->get_logger(), "Stopping");
            } else {
              //Check back/forward
              if(msg->linear.x > 0 && mThrottle <= 0)
              {
                bStop = false;
                //reset throttle if going from backward to forward
                mThrottle = THROTTLE_MIN;
                RCLCPP_INFO(this->get_logger(), "Forward, throttle = %.2f", mThrottle);
              } else if(msg->linear.x > 0 && mThrottle > 0){
                //If the directionality is positive, but the throttle is also positive
                //we just need to increment/decrement the throttle
                if(msg->linear.x > prev_x)
                {
                  //throttle goes up if values increase
                  mThrottle+=THROTTLE_STEP;
                } else {
                  mThrottle-=THROTTLE_STEP;
                }
                
                //check lmits against top
                mThrottle = std::min(float(mThrottle), float(THROTTLE_MAX));
                RCLCPP_INFO(this->get_logger(), "increasing throttle to %.2f", mThrottle);
              } else if(msg->linear.x < 0 && mThrottle >=0)
              {
                bStop = false;
                //reset throttle if going from forward to backward (or stop to back)
                mThrottle = -1.0*THROTTLE_MIN;
                RCLCPP_INFO(this->get_logger(), "Backward, throttle = %.2f", mThrottle);
              } else if(msg->linear.x < 0 && mThrottle < 0) {
                //Directionality is negative, and the throttle is still negative
                //decrease the throttle

                if(msg->linear.x < prev_x)
                {
                  //If it gets more negative make throttle closer to -1
                  mThrottle-=THROTTLE_STEP;
                } else {
                  mThrottle+=THROTTLE_STEP;
                }
                //check limits
                mThrottle = std::max(float(mThrottle), float(-1.0*THROTTLE_MAX));
                RCLCPP_INFO(this->get_logger(), "decreasing throttle to %.2f", mThrottle);
              } else {
                RCLCPP_INFO(this->get_logger(), "Unknown case, throttle is %.2f", mThrottle);
              }
            }
            prev_x = msg->linear.x;
	     
            if(adjustSettings()) {
                RCLCPP_INFO(this->get_logger(), "Message sent correctly");
		auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
                servoMsg.angle = mSteering;
                servoMsg.throttle = mThrottle;
		mServoPub->publish(servoMsg);                                 	
            }
        }


        bool adjustSettings() 
        {
            RCLCPP_INFO(this->get_logger(), "Adjusting Settings");
            if(bStop)
            {
                if(bServoGPIOOn)
                {
                    //turn off the GPIO
                    bServoGPIOOn = false;
		    if(!disableGPIO()){return false;}
			    
                }
            }
            if(!bStop)
            {
                if(!bServoGPIOOn)
                {
                    //turn on the GPIO (off)
                    bServoGPIOOn = true;
		    if(!enableGPIO()){return true;}
                }
            }


            return true;
        }

	bool enableGPIO(){
		auto servo_gpio_request = std::make_shared<deepracer_interfaces_pkg::srv::ServoGPIOSrv::Request>();
		servo_gpio_request->enable = 0;
		while(!mServoGPIOClient->wait_for_service(1s)){
			if(!rclcpp::ok()){
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting, exiting!");
				return false;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, continuing the wait...");
		}	
		auto result = mServoGPIOClient->async_send_request(servo_gpio_request);
		return true;

	}

	bool disableGPIO(){
		auto servo_gpio_request = std::make_shared<deepracer_interfaces_pkg::srv::ServoGPIOSrv::Request>();
		servo_gpio_request->enable = 1;
		while(!mServoGPIOClient->wait_for_service(1s)){
			if(!rclcpp::ok()){
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting, exiting!");
				return false;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, continuing the wait...");
		}	
		auto result = mServoGPIOClient->async_send_request(servo_gpio_request);
		return true;

	}

	rclcpp::Client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>::SharedPtr mServoGPIOClient;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mSubscription;
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr mServoPub;
	bool bStop = true;
	bool bServoGPIOOn = false;
	float mThrottle = 0.0; //start at 50%
	float mSteering = 0;
        float prev_x = 0.5;//artifact of teleop_twist_keyboard


        //Constant values
        const float THROTTLE_STEP = .01;
        const float THROTTLE_MIN = .7;
        const float THROTTLE_MAX = 1;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SshDriver>());
	rclcpp::shutdown();
	return 0;
}
