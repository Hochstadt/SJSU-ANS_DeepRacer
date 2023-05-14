import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


#message type to convey velocity is vector3
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from deepracer_interfaces_pkg.srv import ServoGPIOSrv


from time import sleep
from datetime import datetime
import numpy as np
import math
from .PID import PID
from scipy import integrate
from scipy.spatial.transform import Rotation as R


class velController(Node):
    SAMPLE_TIME =.1 
    def __init__(self):
        super().__init__('vel_controller')

        
        self.bCalibrated = False
        self.bHaveData = False
        self.calibration_time = 10 #seconds
        self.bCmdReceived = False 
        bIMU = False
        self.declare_parameter('bIMU', bIMU)
        self.bIMU = self.get_parameter('bIMU').get_parameter_value().bool_value
        #Helpful parameters!
        velKp = 0
        velKi = 0
        velKd = 0
        rotKp = 0
        rotKi = 0
        rotKd = 0
        self.declare_parameter('velKp', velKp)
        self.declare_parameter('velKi', velKi)
        self.declare_parameter('velKd', velKd)
        self.declare_parameter('rotKp', rotKp)
        self.declare_parameter('rotKi', rotKi)
        self.declare_parameter('rotKd', rotKd)
        #Get parameters
        velKp = self.get_parameter('velKp').get_parameter_value().double_value
        velKi = self.get_parameter('velKi').get_parameter_value().double_value
        velKd = self.get_parameter('velKd').get_parameter_value().double_value
        rotKp = self.get_parameter('rotKp').get_parameter_value().double_value
        rotKi = self.get_parameter('rotKi').get_parameter_value().double_value
        rotKd = self.get_parameter('rotKd').get_parameter_value().double_value


        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        #Need a listener to get the commands
        self.cmd_subscriber = self.create_subscription(Float32MultiArray,
                                                        '/controller/cmd',
                                                        self.cmd_listener, 
                                                        qos_profile=qos_profile)
        
        #Subscribe to IMU, IMU Parameters
        self.xacc = []
        self.xrotvel = []
        self.yacc = []
        self.yrotvel = []
        self.zacc = []
        self.zrotvel = []
        self.acc_time = []
        self.imu_subscriber = self.create_subscription(Imu,
                                                       '/data_raw', 
                                                       self.imu_receiver,
                                                       1)
        
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        self.time_arr = []
        self.x_car = []
        self.y_car = []
        self.theta_world = []
        self.prev_time = [] 
        self.prev_tcar_ref = -1
        self.pose_subscriber = self.create_subscription(PoseStamped,
                                                        '/localization/pose',
                                                        self.pose_receiver, 
                                                        qos_profile=qos_profile)
        self.waypoint_heading = -1
        self.waypoint_subscriber = self.create_subscription(Float32,
                                                        '/controller/waypoint_heading',
                                                        self.waypoint_heading_receiver, 
                                                        qos_profile=qos_profile)
        
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

        self.start_time = -1
        if self.bIMU == True:
            #vPID
            vP = .7 
            vI = 0.03 
            vD = 0.001 
            self.vel_pid = PID(vP, vI, vD)
            self.vel_pid.setSampleTime(self.SAMPLE_TIME)
            self.vel_pid.setWindup(15)
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
        else:
            #vPID
            self.prev_vel = 0
            #force accept cmd
            self.prev_increment = 1000
            self.mThrottle = 0
            vP = .5
            vI = 0.0008
            vD = 0.000 
            self.vel_pid = PID(velKp, velKi, velKd)
            self.vel_pid.setSampleTime(self.SAMPLE_TIME)
            #seems to increase with velocity command rate
            # 2 - 1 second commandnig/update
            self.constant_theta_mult = 2*5 
            self.total_theta = 0
            #self.tracked_vel = 0

            #angvelPID
            avP = 2.0
            avI = 0.005
            avD = 0.000
            self.angvel_pid = PID(rotKp, rotKi, rotKd)
            self.angvel_pid.setSampleTime(self.SAMPLE_TIME)
            self.mSteering = 0
    def waypoint_heading_receiver(self, msg):
        self.waypoint_heading = msg.data
        self.get_logger().info('Waypoing: %.4f' % self.waypoint_heading)

    def pose_receiver(self, msg):
        if self.bIMU == False:
            #I think for this following commetned block to work you'd have
            #to not re-do the rotation as well and then keep track of how
            #that change sin reference tothe origin's roation
            #if any(self.origin) == False:
                #if origin not defined then do so (note 
                # the origin will be constantly redefined where it will
                # always be the start of the way point. )



            tcar_ref = msg.pose.position
            tcar_ref = np.array([tcar_ref.x, tcar_ref.y, tcar_ref.z]) 
            car_q = msg.pose.orientation
            car_q = np.array([car_q.x, car_q.y, car_q.z, car_q.w])
            #Convert to the rot obj
            rcar = R.from_quat(car_q)
            #Get out DCMs
            Rref_car = rcar.as_matrix()

            if self.bHaveData == True and self.waypoint_heading != -1:

                dtime = (datetime.now() - self.prev_time).total_seconds()

                deltat_car = np.matmul(Rref_car.transpose(), self.prev_tcar_ref - tcar_ref)
                meas_vel_x_car = abs(deltat_car[0])/dtime

                #compare self.prev_increment to what the speed change is detected as

                #if abs(meas_vel_x_car - self.prev_vel) > 3 * abs(self.prev_increment):
                #    self.get_logger().info('Command rejected as delta vel is %.4f and current increment is # * %.4f' %
                #        (abs(meas_vel_x_car - self.prev_vel), self.prev_increment))
                if False:
                    
                #    self.get_logger().info('Measured Velocity: <%.4f, %.4f>, Commanded Velocity <%.4f>' % (meas_vel_x_car, meas_vel_y_car, self.vel_pid.SetPoint))
                #    self.get_logger().info('Measured Rotation: <%.4f>, Commanded Rotation <%.4f>' % (np.rad2deg(meas_theta_world), np.rad2deg(self.angvel_pid.SetPoint)))
                    #bad command just decrease throttle in case moving too fast, to get to a place where we understand velocity again
                    tmpmThrottle = -.1
                    #This concept doesn't apply for steering as 0 is cruise, whereas throttle set speed is cruise
                    #tmpmSteering = 0
                else:
                    if self.bCmdReceived == True:
                        self.get_logger().info('Measured Velocity: <%.4f>, Commanded Velocity <%.4f>' % (meas_vel_x_car, self.vel_pid.SetPoint))
                        self.get_logger().info('Measured Rotation: <%.4f>, Commanded Rotation <%.4f>' % 
                                               (np.rad2deg(self.waypoint_heading), np.rad2deg(self.angvel_pid.SetPoint)))

                        #Update the PID
                        #vel_err = self.vel_pid.update(float(meas_vel_x_car))
                        vel_err = self.vel_pid.update(float(meas_vel_x_car))
                        #To make the correct mapping okay, have to add -1
                        rot_err = self.angvel_pid.update(float(-1.0*self.waypoint_heading))
                        self.get_logger().info('Velocity error <%.4f>, Rotation error <%.4f>' % (vel_err, np.rad2deg(rot_err)))

            
                        tmpmThrottle = self.vel_pid.output
                        self.prev_increment = tmpmThrottle
                        self.prev_vel = meas_vel_x_car
                        self.mSteering = self.angvel_pid.output
                        self.mThrottle = self.vel_pid.output
                if self.bCmdReceived == True:
                    if tmpmThrottle is not None:
                        self.get_logger().info('Controller output, throttle inc %.4f, rotation %.4f' % (tmpmThrottle, self.mSteering))
                        #self.mThrottle+=tmpmThrottle 
                        servoMsg = ServoCtrlMsg()
                        if self.mThrottle > 1.0:
                            self.mThrottle = 1.0
                        elif self.mThrottle < 0.0:
                            #Assuming no backwards!
                            self.mThrottle = float(0.0)
                        
                        if self.mSteering > 1.0:
                            self.mSteering = 1.0
                        elif self.mSteering < -1.0:
                            self.mSteering = -1.0
                        self.mSteering = self.mSteering * 1.0
                        servoMsg.throttle = self.mThrottle
                        self.get_logger().info('Setting throttle to %.4f and steering to %.4f' % (self.mThrottle, self.mSteering))
                        servoMsg.angle = self.mSteering
                        self.throttle_pub.publish(servoMsg)
                else:
                    self.get_logger().info('mThrottle is none')
                self.prev_vel = meas_vel_x_car

            #else:
            #need to define prev items
            self.prev_time = datetime.now()
            self.bHaveData = True            
            self.prev_tcar_ref = tcar_ref

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

    def imu_receiver(self, msg):
        if self.bIMU == True:
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


    def cmd_listener(self, msg):
        
        self.get_logger().info('Incoming Command Received: <%.4f> AND <%.4f>' % (msg.data[0], msg.data[1]))
        self.bCmdReceived = True
        #I suppose + rotation is left and - is right
        
        #duration = datetime.now() - self.start_time 
        if self.bIMU == True and self.bCalibrated == True and self.bHaveData == True:
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

        elif self.bHaveData == True and self.bIMU == False:
            
            #Get commands and hit the set-points
            cmd_vel = float(msg.data[0])
            if abs(cmd_vel) > 0 and self.bGPIOEnable == False:
                self.bGPIOEnable = True
                self.enable_gpio()
            
            if cmd_vel == 0:
                self.bGPIOEnable = False
                self.disable_gpio()
            cmd_angle = float(msg.data[1])

            self.vel_pid.SetPoint = cmd_vel
            self.angvel_pid.SetPoint = cmd_angle

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
        sleep(5)

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
