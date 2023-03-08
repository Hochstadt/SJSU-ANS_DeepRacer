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
	    /* 2/18: Come back to finish this spot. Right now I have done the following:
	     * 1) ssh_driver launcher script now launches all needed nodes
	     * 2) X windows forwarding for access through ssh (see the README note section for more details)
	     * 3) ssh source script to source the relevant things
	     * when you luanch (from sudo) the ssh_launch script this should do everything
	     * correctly. And to test the LED light on the car will come on and off
	     * as the servoGPIO is enabled/dispabled
	     * Next: the throttle and steering and subscribions of the servo node, so replicated
	     * what the ctrl node does with it's publishing and integrate it into thie lofic below
	     */


        }

    private:
        void acceptCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Command Received");
            RCLCPP_INFO(this->get_logger(), "Linear: <%.2f, %.2f, %.2f>",
                msg->linear.x, msg->linear.y, msg->linear.z );
            RCLCPP_INFO(this->get_logger(), "Angular: <%.2f, %.2f, %.2f>", 
                msg->angular.x, msg->angular.y, msg->angular.z);

            if(msg->linear.x > 0){
                bStop = false;
                mThrottle+=.1;
		mThrottle = std::min(float(mThrottle), float(1.0));
                RCLCPP_INFO(this->get_logger(), "Setting throttle to %.2f", mThrottle);
            } else if(msg->linear.x < 0){
                bStop = false;
                mThrottle+=-.1;
		mThrottle = std::max(float(mThrottle), float(-1.0));
                RCLCPP_INFO(this->get_logger(), "Setting throttle to %.2f", mThrottle);
            } else if(msg->angular.z < 0){
		mSteering +=-.3;
		mSteering = std::max(float(mSteering), float(-1.0));
		RCLCPP_INFO(this->get_logger(), "Turning left, steering: %.2f", mSteering);
            } else if(msg->angular.z > 0){
		mSteering+=.3;
		mSteering = std::min(float(mSteering), float(1.0));
		RCLCPP_INFO(this->get_logger(), "Turning right, steering: %.2f", mSteering);
            } else if(0.0 == abs(msg->linear.x) + abs(msg->linear.y) + abs(msg->linear.z) + 
                abs(msg->angular.x) + abs(msg->angular.y) + abs(msg->angular.z)) {
		mThrottle = 0;
                bStop = true;
                RCLCPP_INFO(this->get_logger(), "Stopping");
            }
	     
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
	float mThrottle = .5; //start at 50%
	float mSteering = 0;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SshDriver>());
	rclcpp::shutdown();
	return 0;
}
