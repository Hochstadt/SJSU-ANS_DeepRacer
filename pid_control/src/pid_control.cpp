#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "deepracer_interfaces_pkg/srv/servo_gpio_srv.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>
#include <cstdio>
#include <memory>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <cstdlib>


using namespace std::chrono_literals;
using std::placeholders::_1;


namespace keyboard
{
    enum Directions {up = 0, down = 1, right = 2, left = 3, unk=-1};
}

class PIDControl : public rclcpp::Node
{
    public:

	PIDControl() : Node("pid_control")
        {
	    mSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data_raw", 10, std::bind(&PIDControl::acceptCmd, this, _1));
	    auto ServoGPIOClient = this->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>("/servo_pkg/servo_gpio");
            auto servo_gpio_request = std::make_shared<deepracer_interfaces_pkg::srv::ServoGPIOSrv::Request>();
	    sleep(5);
	    servo_gpio_request->enable = 0;
	    ServoGPIOClient->async_send_request(servo_gpio_request);
	    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            //besteffort - will try to deliver samples, but may lose them if network is bad
            qos.reliable();

            x_total=0;
            y_total=0;

	    x_calib=0;
	    y_calib=0;
	    z_calib=0;
	    x_calib_rolling=0;
            y_calib_rolling=0;
            z_calib_rolling=0;


	    calib_time=0;

	    x_rot=0;
	    y_rot=0;
	    z_rot=0;

	    target=0;
	    integ=0;
	    old_error=0;
	    throt=.1;
	    est_speed=0;
	    est_move=0;
	    last_accel=0;
	    last_stop=0;
	    z_turn=0;
	    angle=0;
	    rot_calib=0;
	    rot_calib_rolling=0;


            mServoPub = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", qos);
	    sleep(5);
	    auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
	    servoMsg.throttle=0;
            servoMsg.angle=0;
	    mServoPub->publish(servoMsg);
	}


private:
        void acceptCmd(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
		
		calib_time++;
		//find our gravity and direction
		if(calib_time<=1800)
		{
			if(calib_time%10==0)
			{
				if(abs(msg->linear_acceleration.y<1)&&abs(msg->linear_acceleration.y-y_calib_rolling)<.1&&abs(last_accel-y_calib_rolling)<.1)
				{
					y_calib=y_calib_rolling;
					last_stop=calib_time;
					rot_calib=rot_calib_rolling;

				}
				y_calib_rolling=0;
				
			}
			y_calib_rolling+=msg->linear_acceleration.y/10;
			rot_calib_rolling+=msg->angular_velocity.z/10;
			last_accel=msg->linear_acceleration.y;
		}
		/*
		if(calib_time==1000)
		{
			target=30;
		}
		else if (calib_time==1100)
		{
			target=40;
		}
		else if(calib_time==1200)
		{
			target=50;
		}
		else if(calib_time==1300)
		{
			target=0;
		}*/
		target=20-(calib_time/40);
			est_speed+=msg->linear_acceleration.y-y_calib;
			if(est_speed<0)
			{
				est_speed=0;
			}
			est_move+=est_speed;
			double Kp=.0005;
			double Kd=1.5;
			double Ki=0;
			double error=target-est_speed;
			double P=Kp*error;
			integ+=error/100;
			double I=Ki*integ;
			double D=(Kd*(error-old_error))/100;
			double PID=P+I+D;
			auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
			throt+=PID;
			RCLCPP_INFO(this->get_logger(),"%f,%f,%f,%f,%f",target,est_speed,throt,z_turn,angle);
			if(throt>1)
			{
				throt=1;
			}
			else if (throt<.1)
			{
				throt=.1;
			}
			if(est_speed>3)
			{
				z_turn+=msg->angular_velocity.z;
				double ang_error=z_turn;
				angle+=ang_error*.0001;
				if(angle<-1)
				{
					angle=-1;
				}
				else if (angle>1)
				{
					angle=1;
				}
			}
			else
			{
				angle=0;
			}

                        servoMsg.throttle=throt;
                        servoMsg.angle=angle;
                        mServoPub->publish(servoMsg);
			old_error=error;

		if (calib_time>800)
		{
			auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
			servoMsg.throttle=0;
			servoMsg.angle=0;
			mServoPub->publish(servoMsg);
			RCLCPP_INFO(this->get_logger(),"end");
			exit(0);
		}
	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mSubscription;
	rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr mServoPub;
	double x_calib;
	double y_calib;
	double z_calib;
        double x_calib_rolling;
        double y_calib_rolling;
        double z_calib_rolling;
	double last_accel;
	double last_stop;
	double z_turn;
	double angle;
	double rot_calib;
	double rot_calib_rolling;

	double x_total;
	double y_total;
	double x_rot;
	double y_rot;
	double z_rot;
	int calib_time;
	double target;
	double integ;
	double old_error;
	double throt;
	double est_speed;
	double est_move;

};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PIDControl>());
	rclcpp::shutdown();
	return 0;
}
