#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "deepracer_interfaces_pkg/srv/servo_gpio_srv.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <chrono>
#include <cstdio>
#include <memory>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include "tf2/LinearMath/Transform.h"
using namespace std::chrono_literals;
using std::placeholders::_1;

class PIDControl : public rclcpp::Node
{
    public:

	PIDControl() : Node("pid_control")
        {
	    mIMUSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data_raw", 10, std::bind(&PIDControl::acceptImu, this, _1));
	    mPositionSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
	        "/localization/pose", 10, std::bind(&PIDControl::acceptPose, this, _1));
	    mRouteSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, std::bind(&PIDControl::acceptRoute, this, _1));
	    auto ServoGPIOClient = this->create_client<deepracer_interfaces_pkg::srv::ServoGPIOSrv>("/servo_pkg/servo_gpio");
            auto servo_gpio_request = std::make_shared<deepracer_interfaces_pkg::srv::ServoGPIOSrv::Request>();
	    sleep(5);
	    servo_gpio_request->enable = 0;
	    ServoGPIOClient->async_send_request(servo_gpio_request);
	    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            //besteffort - will try to deliver samples, but may lose them if network is bad
            qos.reliable();

	    Path.reset();

	    path_index=0;

	    y_calib=0;

	    z_rot=0;

	    //TODO make target variable
	    target=0;
	    target_angle=0;
	    integ=0;
	    old_error=0;
	    throt=.3;
	    est_speed=0;
	    est_move=0;
	    est_angle=0;
	    angle=0;

	    pose_recv=false;



            mServoPub = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>("/ctrl_pkg/servo_msg", qos);
	    sleep(5);
	    auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
	    servoMsg.throttle=0;
            servoMsg.angle=0;
	    mServoPub->publish(servoMsg);
	}


private:
        void acceptPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		double roll, pitch, yaw;
		if(pose_recv)
		{
			double delta_x=last_pose.pose.position.x-msg->pose.position.x;
			double delta_y=last_pose.pose.position.y-msg->pose.position.y;
			double movement=sqrt(delta_x*delta_x+delta_y*delta_y);
			double time=msg->header.stamp.nanosec-last_pose.header.stamp.nanosec;
			time=time/1000000000;
			if(msg->header.stamp.sec!=last_pose.header.stamp.sec)
			{
				time+=1;
			}
			//TODO guess at error
			est_speed=movement/time;
			tf2::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
			tf2::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);
			est_angle=yaw;

		}
		else
		{
			pose_recv=true;
		}
		last_pose=*msg;

		if(Path!=nullptr)
		{
			target=.3;
			double delta_x=Path->pose.position.x-msg->pose.position.x;
                        double delta_y=Path->pose.position.y-msg->pose.position.y;
			double distance=sqrt(delta_x*delta_x+delta_y*delta_y);
			target_angle=atan2(delta_y,delta_x);
			if(abs(target_angle-est_angle)>4)
			{
				double new_ang=target_angle+M_PI+(M_PI-est_angle);
				if(target_angle<0)
				{
					target_angle=new_ang;
				}
				else
				{
					target_angle=-new_ang;
				}
				est_angle=0;
			}

			RCLCPP_INFO(this->get_logger(),"Target angle %f, current angle %f, distance %f",target_angle,est_angle,distance);
			RCLCPP_INFO(this->get_logger(),"%f %f %f",roll,pitch,yaw);
			if(distance<.2)
			{
				Path=nullptr;
				target=0;
			}
			
		}
	}

	void acceptRoute(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
		RCLCPP_INFO(this->get_logger(),"got route");
		Path=msg;
	}

        void acceptImu(const sensor_msgs::msg::Imu::SharedPtr msg)
        {
		est_speed+=(msg->linear_acceleration.y-y_calib)/60;
		//TODO move backward
		if(est_speed<0)
		{
			est_speed=0;
		}
		est_move+=est_speed;
		double Kp=.01;
		double Kd=5;
		double Ki=0;
		double error=target-est_speed;
		double P=Kp*error;
		integ+=error/60;
		double I=Ki*integ;
		double D=(Kd*(error-old_error))/60;
		double PID=P+I+D;
		auto servoMsg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
		throt+=PID;
		RCLCPP_INFO(this->get_logger(),"%f,%f,%f,%f,%f",target,est_speed,throt,est_angle,target_angle);
		if(throt>1)
		{
			throt=1;
		}
		else if (throt<.3)
		{
			throt=.3;
		}
		est_angle+=(msg->angular_velocity.z-rot_calib)/60;
		double ang_error=target_angle-est_angle;
		//TODO improve angle PID
		angle=ang_error;
		if(angle<-1)
		{
			angle=-1;
		}
		else if (angle>1)
		{
			angle=1;
		}
		if(target==0)
		{
			angle=0;
		}
		RCLCPP_INFO(this->get_logger(),"%f,%f,%f,%f,%f,%f",target,est_speed,throt,est_angle,target_angle,angle);

		servoMsg.throttle=throt;
		servoMsg.angle=angle;
		mServoPub->publish(servoMsg);
		old_error=error;

	}

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mIMUSubscription;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mPositionSubscription;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mRouteSubscription;
	rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr mServoPub;

	geometry_msgs::msg::PoseStamped::SharedPtr Path;
	unsigned int path_index;
	double y_calib;
	
	double est_angle;
	double angle;
	double rot_calib;

	double z_rot;
	
	double target;
	double target_angle;
	double integ;
	double old_error;
	double throt;
	double est_speed;
	double est_move;
	geometry_msgs::msg::PoseStamped last_pose;
	bool pose_recv;

};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PIDControl>());
	rclcpp::shutdown();
	return 0;
}
