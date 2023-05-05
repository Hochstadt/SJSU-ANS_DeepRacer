import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


#message type to convey velocity is vector3
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import ServoGPIOSrv



from datetime import datetime
import numpy as np
import math
from .PID import PID
from scipy import integrate


class velController(Node):
    SAMPLE_TIME =.5 
    def __init__(self):
        super().__init__('vel_controller')

        self.xacc = []
        self.xrotvel = []
        self.yacc = []
        self.yrotvel = []
        self.zacc = []
        self.zrotvel = []
        self.acc_time = []
        self.bCalibrated = False
        self.calibration_time = 10 #seconds
        #vPID
        self.startingThrottle = .2
        vP = .7 
        vI = 0.03 
        vD = 0.001 
        self.vel_pid = PID(vP, vI, vD)
        self.vel_pid.setSampleTime(self.SAMPLE_TIME)
        self.vel_pid.setWindup(15)
        #self.mThrottle = -10000
        #seems to increase with velocity command rate
        # 2 - 1 second commandnig/update
        self.constant_theta_mult = 2*5 
        self.total_theta = 0
        #self.tracked_vel = 0

        #angvelPID
        avP = 10
        avI = 1
        avD = 1
        self.angvel_pid = PID(avP, avI, avD)
        self.angvel_pid.setSampleTime(self.SAMPLE_TIME)
        self.mSteering = 0


        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )

        #Need a listener to get the velocity commands
        self.vel_subscriber = self.create_subscription(Float32MultiArray,
                                                        '/controller/vel',
                                                        self.vel_listener, 
                                                        qos_profile=qos_profile)
        
        #Subscribe to IMU
        self.bHaveData = False
        self.imu_subscriber = self.create_subscription(Imu,
                                                       '/data_raw', 
                                                       self.imu_receiver,
                                                       1)
        #The be all publisher
        self.throttle_pub = self.create_publisher(
                 ServoCtrlMsg,
                 '/ctrl_pkg/servo_msg',
                 1
            )
        
        self.bGPIOEnable = False
        self.gpio_client = self.create_client(
            ServoGPIOSrv, 
            '/servo_pkg/servo_gpio')
        #while not self.gpio_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service not available...')


        self.start_time = -1
        #Need a calibration for steady motion here
        #basically read the data for 10s, create mean and std to establsih the noise
        #floor
        #Can then apply that to each measurement
        
        #IMU stuff:
        #self.get_logger().info(f"Trying to initialize the sensor at {constants.BMI160_ADDR} on bus {constants.I2C_BUS_ID}")

        #Go turn on the GPIOO
        

    def imu_receiver(self, msg):
        #self.get_logger().info('IMU Message Received')
        #First check if we're calibrated
        #the acc is supposed to do this but it really didn't so...
        if self.bCalibrated:
            #Get accelerometer data
            #save it to the accel array
            #self.acc_time.append(msg.header.stamp)
            dtime = datetime.now() - self.start_time
            self.acc_time.append(dtime.total_seconds())
            self.xacc.append(msg.linear_acceleration.x)
            self.xrotvel.append(msg.angular_velocity.x)
            self.yacc.append(msg.linear_acceleration.y)
            self.yrotvel.append(msg.angular_velocity.y)
            self.zacc.append(msg.linear_acceleration.z)
            self.zrotvel.append(msg.angular_velocity.z)
            self.bHaveData = True

        else:
            #Calibrate
            if self.start_time == -1:
                self.start_time = datetime.now()

            duration = datetime.now() - self.start_time

            if duration.total_seconds() < self.calibration_time:
                #accumulate data
                #self.acc_time.append(msg.header.stamp)
                dtime = datetime.now() - self.start_time
                self.acc_time.append(dtime.total_seconds())
                self.xacc.append(msg.linear_acceleration.x)
                self.xrotvel.append(msg.angular_velocity.x)
                self.yacc.append(msg.linear_acceleration.y)
                self.yrotvel.append(msg.angular_velocity.y)
                self.zacc.append(msg.linear_acceleration.z)
                self.zrotvel.append(msg.angular_velocity.z)

            else:
                #do the calibration
                np_xacc = np.array(self.xacc)
                self.xacc_mean = np.mean(np_xacc)
                self.xacc_std = np.std(np_xacc)
                np_xrotvel = np.array(self.xrotvel)
                self.xrotvel_mean = np.mean(np_xrotvel)
                self.xrot_vel_std = np.std(np_xrotvel)

                np_yacc = np.array(self.yacc)
                self.yacc_mean = np.mean(np_yacc)
                self.yacc_std = np.std(np_yacc)
                np_yrotvel = np.array(self.yrotvel)
                self.yrotvel_mean = np.mean(np_yrotvel)
                self.yrotvel_std = np.std(np_yrotvel)

                np_zacc = np.array(self.zacc)
                self.zacc_mean = np.mean(np_zacc)
                self.zacc_std = np.std(np_zacc)
                np_zrotvel = np.array(self.zrotvel)
                self.zrotvel_mean = np.mean(np_zrotvel)
                self.zrotvel_std = np.std(np_zrotvel)

                self.xacc = []
                self.xrotvel = []
                self.yacc = []
                self.yrotvel = []
                self.zacc = []
                self.zrotvel = []
                self.acc_time = []
                self.bCalibrated = True
                self.bHaveData = False
                self.start_time = datetime.now()


    def vel_listener(self, msg):
        #self.get_logger.info('Velocity Command Received: <%.4f %.4f %.4f>' % (msg.x, msg.y, msg.z))
        self.get_logger().info('Incoming Command Received: <%.4f> AND <%.4f>' % (msg.data[0], msg.data[1]))

        
        #duration = datetime.now() - self.start_time 
        if self.bCalibrated == True and self.bHaveData == True:
            #ideally specified in the body frame of the car, so X = forward?
            #need to verify with IMU output to make sure...
            #cmd_vel_x = msg.x
            #cmd_vel_y = msg.y
            #cmd_vel_z = msg.z

            cmd_vel = float(msg.data[0])

            if abs(cmd_vel) > 0 and self.bGPIOEnable == False:
                self.bGPIOEnable = True
                self.enable_gpio()
            
            if cmd_vel == 0:
                self.bGPIOEnable = False
                self.disable_gpio()

            cmd_angle = float(msg.data[1])

            self.vel_pid.SetPoint = cmd_vel
            ##Identify current velocity:
            #Take off the offset for the mean (should have a check on the std to verify validity)
            self.get_logger().info('Length acc = %d, length angv = %d' % (len(self.xacc), len(self.xrotvel)))
            np_xacc = np.array(self.xacc) - self.xacc_mean
            #Reduce via std noise floor
            #np_xacc[abs(np_xacc) > self.xacc_std] = 0 
            np_yacc = np.array(self.yacc) - self.yacc_mean
            #np_yacc[abs(np_yacc) > self.yacc_std] = 0
            np_zacc = np.array(self.zacc) - self.zacc_mean
            #np_zacc[abs(np_zacc) > self.zacc_std] = 0

            #np_xrotvel = np.array(self.xrotvel) - self.xrotvel_mean
            #np_yrotvel = np.array(self.yrotvel) - self.yrotvel_mean
            #np_zrotvel = np.array(self.zrotvel) - self.zrotvel_mean
            np_xrotvel = np.array(self.xrotvel)
            np_yrotvel = np.array(self.yrotvel)
            np_zrotvel = np.array(self.zrotvel)

            #init_time = float(self.acc_time[0].nanosec/10**9)
            init_time = self.acc_time[0]
            self.get_logger().info('Acceleration Info')
            self.get_logger().info('Mean: <%.4f, %.4f, %.4f>' % (self.xacc_mean, self.yacc_mean, self.zacc_mean))
            self.get_logger().info('OgMean <%.4f, %.4f, %.4f>' % (np.mean(np.array(self.xacc)), np.mean(np.array(self.yacc)), np.mean(np.array(self.zacc))))
            self.get_logger().info('Rotation Velocity Info')
            self.get_logger().info('Mean <%.8f, %.8f, %.8f>' % (self.xrotvel_mean, self.yrotvel_mean, self.zrotvel_mean))


            #if np.mean(np_xacc) > self.xacc_std or np.mean(np_yacc) > self.yacc_std or np.mean(np_zacc) > self.zacc_std:
            #self.get_logger().info('Delta times shape: %d' % (delta_times.shape[0]))
            delta_times = []
            for i in range(0, len(self.acc_time)):
                #dtime = float(self.acc_time[i].nanosec/10**9)
                dtime = self.acc_time[i]
                delta_times.append(dtime - init_time)
            #linear velocity
            #self.get_logger().info('Init Time: %.4f, end time: %.4f' % (init_time, (self.acc_time[-1].nanosec/10**9)))
            self.get_logger().info('Init Time: %.4f, end time: %.4f' % (init_time, self.acc_time[i]))
            delta_times = np.array(delta_times)
            self.get_logger().info('Delta Time Shape %d' %  delta_times.shape[0])
            self.get_logger().info('Len of acc time %d' % len(self.acc_time))
            self.get_logger().info('Acc Shape %d' %  np_xacc.shape[0])
            self.get_logger().info('Angular Velocity Shape %d' % np_xrotvel.shape[0])
            vel_x_meas = integrate.trapezoid(np_xacc, x=delta_times)
            #vel_x_meas = np_xacc[-1] * delta_times[-1]
            #self.tracked_vel += vel_x_meas
            vel_y_meas = integrate.trapezoid(np_yacc, x=delta_times)
            vel_z_meas = integrate.trapezoid(np_zacc, x=delta_times)
            self.get_logger().info('Velocity: <%.4f, %.4f, %.4f>' % (vel_x_meas, vel_y_meas, vel_z_meas))
            #self.get_logger().info('Tracked vel: %.4f' % self.tracked_vel)

            #angular velocity
            deltatheta_x = integrate.trapezoid(np_xrotvel, x=delta_times)
            deltatheta_y = integrate.trapezoid(np_yrotvel, x=delta_times)
            deltatheta_z = integrate.trapezoid(np_zrotvel, x=delta_times)
            #Over the coarse of given time this is the change in orientation
            #the calibration process should have made this start at 0 degrees....
            #deltatheta_x = angvel_x_meas * delta_times[-1] 
            #deltatheta_y = angvel_y_meas * delta_times[-1]
            #deltatheta_z = angvel_z_meas * delta_times[-1]
            #Print delta theta to see what's up:
            self.get_logger().info('Delta time: start: %.8f, end: %.8f' % (delta_times[0],  delta_times[-1]))
            self.get_logger().info('AngVel mean <%.8f, %.8f, %.8f>' % (np.mean(np.array(self.xrotvel)), np.mean(np.array(self.yrotvel)), np.mean(np.array(self.zrotvel))))

            self.get_logger().info('AngVel sum <%.8f, %.8f, %.8f>' % (np.sum(np.array(self.xrotvel)), np.sum(np.array(self.yrotvel)), np.sum(np.array(self.zrotvel))))
            self.get_logger().info('Delta Theta: <%.8f, %.8f, %.8f>' % (deltatheta_x, deltatheta_y, deltatheta_z))
            #self.total_theta += deltatheta_z 
            self.total_theta += delta_times[-1] * np.sum(np.array(self.zrotvel)) * self.constant_theta_mult
            self.get_logger().info('Total Theta: <%.8f>' % self.total_theta)

            #now norm it, and don't really count z ...
            
            #meas_vel = math.sqrt(vel_x_meas**2 + vel_y_meas**2)
            meas_vel = vel_x_meas
            #Need to add the direction, use x to deduce that but only if decernable direction
            #if vel_x_meas < 0 and np.mean(np_xacc) > self.xacc_std:
            #    meas_vel = -1.0 * meas_vel
            
            tmp_err = self.vel_pid.update(float(meas_vel))
            self.get_logger().info('Incoming Velocity Set point <%.4f>, Velocity measured: <%.4f>' % (self.vel_pid.SetPoint, meas_vel))
            self.get_logger().info('Error of PID: <%.4f>' % tmp_err)
            mThrottle = self.vel_pid.output
            if mThrottle is not None:
                #Need to decide to how to handle and control the steering, for now setting mSteering
                # to 0. Like 
                mSteering = 0.0

                servoMsg = ServoCtrlMsg()
                if mThrottle > 1.0:
                    mThrottle = 1.0
                elif mThrottle < -1.0:
                    mThrottle = -1.0
                servoMsg.throttle = mThrottle
                self.get_logger().info('Setting throttle to %.4f' % (mThrottle))
                servoMsg.angle = mSteering
                self.throttle_pub.publish(servoMsg)
            else:
                self.get_logger().info('Mthrottle is none')
            
            #Zero out IMu receipts:
            self.xacc = []
            self.xrotvel = []
            self.yacc = []
            self.yrotvel = []
            self.zacc = []
            self.zrotvel = []
            self.acc_time = []
            self.bHaveData = False
            #self.start_time = datetime.now()



        else:
            self.get_logger().info('Incoming Command Ignored')

    def enable_gpio(self):
        gpio_request = ServoGPIOSrv.Request()
        gpio_request.enable = 0
        #future = self.gpio_client.call_async(gpio_request)
        self.get_logger().info('Trying to enable gpio')
        self.gpio_client.call_async(gpio_request) 
        #rclpy.spin_until_future_complete(gpio_request, future)
        self.get_logger().info('GPIO Enabled')

    def disable_gpio(self):
        self.get_logger().info('Trying to disable gpio')
        gpio_request = ServoGPIOSrv.Request()
        gpio_request.enable = 1
        #future = self.gpio_client.call_async(gpio_request)
        self.gpio_client.call_async(gpio_request)
        #rclpy.spin_until_future_complete(gpio_request, future)
        self.get_logger().info('GPIO Disabled')


def main(args=None):
    rclpy.init(args=args)
    
    vel_controller = velController()
    print('Spinning Node...')
    rclpy.spin(vel_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    print('Destroying Node')
    vel_controller.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
