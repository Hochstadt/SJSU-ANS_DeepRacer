import rclpy
#message type to convey velocity is vector3
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import Time


from datetime import datetime
import numpy as np
import PID
from scipy import integrate


class velController(Node):
    def __init__(self):
        super().__init__('vel_controller')

        self.xacc = []
        self.yacc = []
        self.zacc = []
        self.acc_time = []
        self.bCalibrated = False
        self.calibration_time = 5 #seconds
        #PID
        P = 10
        I = 1
        D = 1
        self.pid = PID.PID(P, I, D)

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )

        #Need a listener to get the velocity commands
        self.vel_subscriber = self.create_subscription(Vector3,
                                                        '/localization/pose',
                                                        self.vel_listener, 
                                                        qos_profile=qos_profile)
        

        self.imu_subscriber = self.create_subscription(Imu,
                                                       '/imu_pkg/data_raw', 
                                                       self.imu_receiver,
                                                       1)
        self.start_time = -1
        #Need a calibration for steady motion here
        #basically read the data for 10s, create mean and std to establsih the noise
        #floor
        #Can then apply that to each measurement
        
        #IMU stuff:
        #self.get_logger().info(f"Trying to initialize the sensor at {constants.BMI160_ADDR} on bus {constants.I2C_BUS_ID}")


    def imu_receiver(self, msg):
        #First check if we're calibrated
        #the acc is supposed to do this but it really didn't so...
        if self.bCalibrated:
            #Get accelerometer data
            #save it to the accel array
            self.acc_time.append(msg.header.stamp)
            self.xacc.append(msg.linear_acceleration.x)
            self.yacc.append(msg.linear_acceleration.y)
            self.zacc.append(msg.linear_acceleration.z)

        else:
            #Calibrate
            if self.start_time == -1:
                self.start_time = datetime.now()
            duration = datetime.now() - self.start_time
            if duration.total_seconds() < self.calibration_time:
                #accumulate data
                self.xacc.append(msg.linear_acceleration.x)
                self.yacc.append(msg.linear_acceleration.y)
                self.zacc.append(msg.linear_acceleration.z)
            else:
                #do the calibration
                np_xacc = np.array(self.xacc)
                self.xacc_mean = np.mean(np_xacc)
                self.xacc_std = np.std(np_xacc)
                np_yacc = np.array(self.yacc)
                self.yacc_mean = np.mean(np_yacc)
                self.yacc_std = np.std(np_yacc)
                np_zacc = np.array(self.zacc)
                self.zacc_mean = np.mean(np_zacc)
                self.zacc_std = np.std(np_zacc)



    def vel_listener(self, msg):
        self.get_logger.info('Velocity Command Received: <%.4f %.4f %.4f>' % (msg.x, msg.y, msg.z))
        
        
        if self.bCalibrated:
            #ideally specified in the body frame of the car, so X = forward?
            #need to verify with IMU output to make sure...
            vel_x = msg.x
            vel_y = msg.y
            vel_z = msg.z

            ##Identify current velocity:
            #Take off the offset for the mean (should have a check on the std to verify validity)
            np_xacc = np.array(self.xacc) - self.xacc_mean
            np_yacc = np.array(self.yacc) - self.yacc_mean
            np_zacc = np.array(self.zacc) - self.zacc_mean

            init_time = self.acc_time[0]
            delta_times = np.zeros((len(self.acc_time)))
            for i in self.acc_time:
                delta_times[i] = (self.acc_time[i] - init_time).seconds
            vel_x_meas = integrate.trapezoid(np_xacc, x=delta_times)
            vel_y_meas = integrate.trapezoid(np_yacc, x=delta_times)
            vel_z_meas = integrate.trapezoid(np_zacc, x=delta_times)



        else:
            self.get_logger.info('Calibration Not Done, Command Ignored')






