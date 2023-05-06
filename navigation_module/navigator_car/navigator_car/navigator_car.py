#ROS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped

#Python
import numpy as np
from time import sleep
from datetime import datetime
from scipy.spatial.transform import Rotation as R

class navigatorCar(Node):
    BOUND_CONSTANT = 3
    def __init__(self):
        super().__init__('navigator_car')
        self.bReceivedPath = False
        self.bNoiseChar = False
        self.avg_vel = .3 
        self.path_index = 1


        self.path_subscriber = self.create_subscription(Path,
                                                '/navigator_car/global_path',
                                                self.path_listener,
                                                10)
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        #This also needs to listen to the pose
        self.pose_subscriber = self.create_subscription(PoseStamped,
                                                        '/localization/pose',
                                                        self.pose_listener, 
                                                        qos_profile=qos_profile)
        
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        
        self.cmd_publisher = self.create_publisher(Float32MultiArray,
                                                   '/controller/cmd', 
                                                   qos_profile=qos_profile)
        timer_period = .5
        #self.timer = self.create_timer(timer_period, self.update_cmd)

        
    def pose_listener(self, msg):
        #Basically just save this so we always have an estimate of this state
        self.current_state = msg.pose
        if self.bReceivedPath == True and self.bNoiseChar == True:
            #Check if we've hit the waypoint
            cur_pos = self.current_state.position
            way_pos = self.next_pose.position
            cur_theta = self.quat_to_theta(self.current_state.orientation)
            way_theta = self.quat_to_theta(self.next_pose.orientation)

            dx = abs(cur_pos.x - way_pos.x)
            dy = abs(cur_pos.y - way_pos.y)
            dtheta = abs(cur_theta - way_theta)
            if dx < self.x_limit and dy < self.y_limit and dtheta < self.theta_limit:
                self.get_logger().info('Achieved Waypoint with the following stats')
                
                if self.path_index >= len(self.path.poses):
                    self.get_logger().error('Error - out of poses in the path, are you sure you are not at the end?')
                else:
                    self.next_pose = self.path.poses[self.path_index].pose
                    self.path_index+=1
        
            self.get_logger().info('Current pos <%.4f, %.4f> %.4f deg vs. Waypoint pos <%.4f, %.4f> %.4f deg' 
                                    % (cur_pos.x, cur_pos.y, np.rad2deg(cur_theta),
                                        way_pos.x, way_pos.y, np.rad2deg(way_theta)))
            self.get_logger().info('Deltas <%.4f, %.4f> %.4f deg' %
                                    (dx, dy, np.rad2deg(dtheta)))

            #update the command            
            cmd_theta = way_theta
            cmd_vel =  self.avg_vel
            cmd = Float32MultiArray()
            cmd.data.append(cmd_vel)
            cmd.data.append(cmd_theta)
            #because we are using the localizer as the measurement we have the global alignmetn, thus we can
            #force our control loop to align in that space making any transofrms or things uncecessary. With that
            #said if a point is way out of the turn radius or something, the car may continually try to turn trying
            #to acheive that way point. There's also no control on the position itself, so we should probably have
            #a timer to indicate a 'missed way point' and move on to the enxt one. This can be done by looking at
            #the speed commanded (and recorded...), and deduce how long it should have taken to get to the waypoint

            self.cmd_publisher.publish(cmd)
                
    def path_listener(self, msg):
        self.path = msg
        if self.bNoiseChar == False:
            start_pose = self.path.poses[0].pose
            #Try to understand the deviations from the current state (as ideally
            # they should be near identical. Can characterize the noise this way)
            dx = self.current_state.position.x - start_pose.position.x
            dy = self.current_state.position.y - start_pose.position.y
            qstart = self.current_state.orientation            
            theta_start = self.quat_to_theta(qstart)
            qnow = start_pose.orientation            
            theta_now = self.quat_to_theta(qnow)
            dtheta = theta_now - theta_start

            #define the noise values
            self.theta_limit = self.BOUND_CONSTANT * abs(dtheta)
            self.x_limit = self.BOUND_CONSTANT * abs(dx)
            self.y_limit = self.BOUND_CONSTANT * abs(dy)
            self.start_pose = start_pose
            self.get_logger().info('Path length identified as %d, indexing at %d' % (len(self.path.poses),self.path_index))
            self.get_logger().info('Bounds as 3 * <%.4f, %.4f> %.4f deg' % (self.x_limit, self.y_limit, np.rad2deg(self.theta_limit)))
            self.next_pose = self.path.poses[self.path_index].pose
            
            #self.projected_time = dx/self.avg_vel
            
            self.path_index+=1
            self.bNoiseChar = True
        
        #deduce the next waypoint as ideally the next one in the chain...        
        self.bReceivedPath = True

    def quat_to_theta(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        e = r.as_euler('zyx')
        return e[0]
    
    #def update_cmd(self, msg):
    #    if self.bReceivedPath == True:

def main(args=None):
    rclpy.init(args=args)
    navigator_car = navigatorCar()
    rclpy.spin(navigator_car)
    navigator_car.destroy_node()
    rclpy.shotdown()

if __name__ == '__main__':
    main()
