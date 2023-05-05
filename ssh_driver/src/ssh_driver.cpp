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


namespace keyboard
{
    enum Directions {up = 0, down = 1, right = 2, left = 3, unk=-1};
}

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
            //Convert incoming messages appropriately
            float ang_x = float(msg->angular.x);
            float ang_y = float(msg->angular.y);
            float ang_z = float(msg->angular.z);
            float lin_x = float(msg->linear.x);
            float lin_y = float(msg->linear.y);
            float lin_z = float(msg->linear.z);
            //lower case k puts this in stop mode
            if(0.0 == abs(lin_x) + abs(lin_y) + abs(lin_z) +
              abs(ang_x) + abs(ang_y) + abs(ang_z)) {
              bStop = true;
              bReverse = false;
              bDrive = false;
            } else if(lin_z > 0) {
              bDrive = true;
              bReverse = false;
              bStop = false;
            } else if(lin_z < 0) {
              bStop = false;
              bDrive = false;
              bReverse = true;
            }

            if(bStop){
                mThrottle = 0; mSteering = 0;
                RCLCPP_INFO(this->get_logger(), "stopping");
            } else if(bDrive){
                RCLCPP_INFO(this->get_logger(), "driving");
                //Starting conditions:
                keyboard::Directions throttle_command = evalInput(lin_x, lin_y, lin_z
                                                        ,ang_x, ang_y, ang_z);
                if(mThrottle == 0.0) {
                    //To start need an 'up' arrow so positive throttle
                    if(throttle_command == keyboard::up) {

                        RCLCPP_INFO(this->get_logger(), "applying iniital throttle");
                        mThrottle = THROTTLE_MIN;
                    }
                } else {
                     //decide if adding or subtracting from current throttle
                     switch(throttle_command){
                         case keyboard::up:
                             //add
                             mThrottle+=THROTTLE_STEP;
                             mThrottle = std::min(mThrottle, THROTTLE_MAX);
                             RCLCPP_INFO(this->get_logger(), "Increasing Throttle");
                             break;
                         case keyboard::down:
                             //Make decrease faster....
                             mThrottle-=3*THROTTLE_STEP;
                             mThrottle = std::max(mThrottle, float(0.0));
                             RCLCPP_INFO(this->get_logger(), "Decreasing Throttle");
                             break;
                         default:
                             break;
                     }
                }
            } else if(bReverse){
                RCLCPP_INFO(this->get_logger(), "reversing");
                keyboard::Directions throttle_command = evalInput(lin_x, lin_y, lin_z,
                                                                ang_x, ang_y, ang_z);
                if(mThrottle == 0.0) {
                    //To start need down arrow to get positive throttle
                    if(throttle_command == keyboard::down) {
                        RCLCPP_INFO(this->get_logger(), "Applying initial reverese");
                        mThrottle = THROTTLE_MIN;
                    }
                } else {
                    switch(throttle_command){
                        case keyboard::down:
                            //down positively increases throttle for reverse
                            mThrottle+=THROTTLE_STEP;
                            mThrottle = std::min(mThrottle, THROTTLE_MAX);
                            RCLCPP_INFO(this->get_logger(), "Increasing Reverse Throttle");
                            break;
                        case keyboard::up:
                            mThrottle-=3*THROTTLE_STEP;
                            mThrottle = std::max(mThrottle, float(0.0));
                            RCLCPP_INFO(this->get_logger(), "Decreasing Reverse Throttle");
                            break;
                        default:
                            break;

                    }
                }
            }

            //Steering
            if(bDrive || bReverse){
                //check if received a steering command
                keyboard::Directions steering_command = evalInput(lin_x, lin_y, lin_z, ang_x, ang_y, ang_z);
                switch(steering_command) {
                    case keyboard::right:
                        //go right
                        mSteering-=STEERING_STEP;
                        mSteering = std::max(mSteering, float(-1.0*STEERING_MAX));
                        RCLCPP_INFO(this->get_logger(), "Veering right");
                        break;
                    case keyboard::left:
                        //go left
                        mSteering+=STEERING_STEP;
                        mSteering = std::min(mSteering, float(STEERING_MAX));
                        RCLCPP_INFO(this->get_logger(), "Veering left");
                        break;
                    default:
                        break;
                }



            }



            RCLCPP_INFO(this->get_logger(), "Throttle %.2f", mThrottle);
            RCLCPP_INFO(this->get_logger(), "Steering %.2f", mSteering);

            if(adjustSettings()) {
                RCLCPP_INFO(this->get_logger(), "Message sent correctly");
                auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
                servoMsg.angle = mSteering;
                if(bReverse){
                    servoMsg.throttle = -1.0*mThrottle;
                } else {
                    servoMsg.throttle = mThrottle;
                }
                // TODO - Uncomment this! Temporary to give me servo control. The logic here for sending servo messages
                //        isn't really compatible with the joystick control in the GUI. Using "cmdvel_to_servo_pkg" for now
                // mServoPub->publish(servoMsg);
            }
        }

        keyboard::Directions evalInput(const float & lin_x, const float & lin_y, const float & lin_z,
            const float & ang_x, const float & ang_y, const float & ang_z)
        {
            //This will check for certain conditions: (should turn into enum)
            if(lin_x > 0.0 && lin_y == 0.0 && lin_z == 0.0 && ang_x == 0.0 && ang_y == 0.0 && ang_z == 0.0){
                //up arrow
                return keyboard::up;
            } else if(lin_x < 0.0 && lin_y == 0.0 && lin_z == 0.0
                && ang_x == 0.0 && ang_y == 0.0 && ang_z == 0.0){
                //down arrow
                return keyboard::down;
            } else if(lin_x == 0.0 && lin_y == 0.0 && lin_z == 0.0
                && ang_x == 0.0 && ang_y == 0.0 && ang_z < 0.0){
                //right arrow
                return keyboard::right;
            } else if(lin_x == 0.0 && lin_y == 0.0 && lin_z == 0.0
                && ang_x == 0.0 && ang_y == 0.0 && ang_z > 0.0){
                //left arrow
                return keyboard::left;
            }

            return keyboard::unk;

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


        keyboard::Directions mDirections;
	bool bStop = true;
        bool bReverse = false;
        bool bDrive = false;
	bool bServoGPIOOn = false;
	float mThrottle = 0.0; //start at 50%
	float mSteering = 0;
        float prev_x = 0.0;//artifact of teleop_twist_keyboard
        float prev_z = 0.0;


        //Constant values
        const float THROTTLE_STEP = .01;
        const float THROTTLE_MIN = .7;
        const float THROTTLE_MAX = 1;
        const float STEERING_STEP = .3;
        const float STEERING_MIN = .3;
        const float STEERING_MAX = 1;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SshDriver>());
	rclcpp::shutdown();
	return 0;
}
