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
    SAMPLE_TIME = .25
    def __init__(self):
        super().__init__('vel_controller')

        self.xacc = []
        self.xrotacc = []
        self.yacc = []
        self.yrotacc = []
        self.zacc = []
        self.zrotacc = []
        self.acc_time = []
        self.bCalibrated = False
        self.calibration_time = 5 #seconds
        #vPID
        vP = 10
        vI = 1
        vD = 1
        self.vel_pid = PID(vP, vI, vD)
        self.vel_pid.setSampleTime(self.SAMPLE_TIME)
        self.mThrottle = 0

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
                                                       '/imu_pkg/data_raw', 
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
        self.get_logger().info('IMU Message Received')
        #First check if we're calibrated
        #the acc is supposed to do this but it really didn't so...
        if self.bCalibrated:
            #Get accelerometer data
            #save it to the accel array
            self.acc_time.append(msg.header.stamp)
            self.xacc.append(msg.linear_acceleration.x)
            self.xrotacc.append(msg.angular_velocity.x)
            self.yacc.append(msg.linear_acceleration.y)
            self.yrotacc.append(msg.angular_velocity.y)
            self.zacc.append(msg.linear_acceleration.z)
            self.zrotacc.append(msg.angular_velocity.z)
            self.bHaveData = True

        else:
            #Calibrate
            if self.start_time == -1:
                self.start_time = datetime.now()

            duration = datetime.now() - self.start_time
            print('duration: ', duration.total_seconds())
            print('cal time: ', self.calibration_time)

            if duration.total_seconds() < self.calibration_time:
                #accumulate data
                self.acc_time.append(msg.header.stamp)
                self.xacc.append(msg.linear_acceleration.x)
                self.xrotacc.append(msg.angular_velocity.x)
                self.yacc.append(msg.linear_acceleration.y)
                self.yrotacc.append(msg.angular_velocity.y)
                self.zacc.append(msg.linear_acceleration.z)
                self.zrotacc.append(msg.angular_velocity.z)

            else:
                #do the calibration
                np_xacc = np.array(self.xacc)
                self.xacc_mean = np.mean(np_xacc)
                self.xacc_std = np.std(np_xacc)
                np_xrotacc = np.array(self.xrotacc)
                self.xrotacc_mean = np.mean(np_xrotacc)
                self.xrot_acc_std = np.std(np_xrotacc)

                np_yacc = np.array(self.yacc)
                self.yacc_mean = np.mean(np_yacc)
                self.yacc_std = np.std(np_yacc)
                np_yrotacc = np.array(self.yrotacc)
                self.yrotacc_mean = np.mean(np_yrotacc)
                self.yrotacc_std = np.std(np_yrotacc)

                np_zacc = np.array(self.zacc)
                self.zacc_mean = np.mean(np_zacc)
                self.zacc_std = np.std(np_zacc)
                np_zrotacc = np.array(self.zrotacc)
                self.zrotacc_mean = np.mean(np_zrotacc)
                self.zrotacc_std = np.std(np_zrotacc)

                self.xacc = []
                self.xrotacc = []
                self.yacc = []
                self.yrotacc = []
                self.zacc = []
                self.zrotacc = []
                self.acc_time = []
                self.bCalibrated = True
                self.bHaveData = False


    def vel_listener(self, msg):
        #self.get_logger.info('Velocity Command Received: <%.4f %.4f %.4f>' % (msg.x, msg.y, msg.z))
        self.get_logger().info('Incoming Command Received: <%.4f> AND <%.4f>' % (msg.data[0], msg.data[1]))
        
        
        if self.bCalibrated == True and self.bHaveData == True:
            #ideally specified in the body frame of the car, so X = forward?
            #need to verify with IMU output to make sure...
            #cmd_vel_x = msg.x
            #cmd_vel_y = msg.y
            #cmd_vel_z = msg.z

            cmd_vel = msg.data[0]

            if abs(cmd_vel) > 0 and self.bGPIOEnable == False:
                self.bGPIOEnable = True
                #self.enable_gpio()
            
            if cmd_vel == 0:
                self.bGPIOEnable = False
                #self.disable_gpio()

            cmd_angle = msg.data[1]

            self.vel_pid.setPoint = cmd_vel
            ##Identify current velocity:
            #Take off the offset for the mean (should have a check on the std to verify validity)
            np_xacc = np.array(self.xacc) - self.xacc_mean
            np_yacc = np.array(self.yacc) - self.yacc_mean
            np_zacc = np.array(self.zacc) - self.zacc_mean

            np_xrotacc = np.array(self.xrotacc) - self.xrotacc_mean
            np_yrotacc = np.array(self.yrotacc) - self.yrotacc_mean
            np_zrotacc = np.array(self.zrotacc) - self.zrotacc_mean

            init_time = self.acc_time[0]
            delta_times = np.zeros((len(self.acc_time)))
            print('Acc time stats')
            print(self.acc_time)
            print(len(self.acc_time))
            print('Delta times shape: ', delta_times.shape)
            for i in range(0, len(self.acc_time)):
                print('i', i)
                print('self.acc_time: ', self.acc_time[i])
                print('init time: ', init_time)
                print('Seconds: ', (self.acc_time[i].sec - init_time.sec))
                delta_times[i] = self.acc_time[i].sec - init_time.sec
            #linear velocity
            print('Integrating')
            vel_x_meas = integrate.trapezoid(np_xacc, x=delta_times)
            vel_y_meas = integrate.trapezoid(np_yacc, x=delta_times)
            vel_z_meas = integrate.trapezoid(np_zacc, x=delta_times)
            #angular velocity
            angvel_x_meas = integrate.trapezoid(np_xrotacc, x=delta_times)
            angvel_y_meas = integrate.trapezoid(np_yrotacc, x=delta_times)
            angvel_z_meas = integrate.trapezoid(np_zrotacc, x=delta_times)
            #Over the coarse of given time this is the change in orientation
            #the calibration process should have made this start at 0 degrees....
            deltatheta_x = angvel_x_meas * delta_times[-1] 
            deltatheta_y = angvel_y_meas * delta_times[-1]
            deltatheta_z = angvel_z_meas * delta_times[-1]
            #Print delta theta to see what's up:
            self.get_logger().info('AngVel: <%.4f, %.4f, %.4f>' % (angvel_x_meas, angvel_y_meas, angvel_z_meas))
            self.get_logger().info('Delta Theta: <%.4f, %.4f, %.4f>' % (deltatheta_x, deltatheta_y, deltatheta_z))

            #now norm it
            meas_vel = math.sqrt(vel_x_meas**2 + vel_y_meas**2 + vel_z_meas**2)
            self.get_logger().info('Velocity Command Received: <%.4f>' % meas_vel)
            mThrottle = self.vel_pid.update(meas_vel)
            print(mThrottle)
            if mThrottle is not None:
                self.get_logger().info('Setting throttle to %.4f' % mThrottle)
                #Need to decide to how to handle and control the steering, for now setting mSteering
                # to 0. Like 
                mSteering = 0

                servoMsg = ServoCtrlMsg()
                servoMsg.throttle = mThrottle
                servoMsg.angle = mSteering
                self.throttle_pub.publish(servoMsg)

            #Zero out IMu receipts:
            self.xacc = []
            self.xrotacc = []
            self.yacc = []
            self.yrotacc = []
            self.zacc = []
            self.zrotacc = []
            self.acc_time = []
            self.bHaveData = False



        else:
            self.get_logger().info('Incoming Command Ignored')

    def enable_gpio(self):
        gpio_request = ServoGPIOSrv.Request()
        gpio_request.enable = 0
        self.future = self.gpio_client.call_async(gpio_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def disable_gpio(self):
        gpio_request = ServoGPIOSrv.Request()
        gpio_request.enable = 1
        self.future = self.gpio_client.call_async(gpio_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


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