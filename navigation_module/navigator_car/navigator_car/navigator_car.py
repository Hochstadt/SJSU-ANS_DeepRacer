#ROS
'''
Notes:
-Increase the gains for the turning OR do more basic logic of soft turn, strong turn, etc. 
 but this seems like your calibration has to be pretty good
-For waypoint checking though, should increase the accpet statement as it seems like 
 we're missing because of rotation
'''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_msgs.msg import Float32

#Python
import numpy as np
from time import sleep
from datetime import datetime
from scipy.spatial.transform import Rotation as R

class navigatorCar(Node):
    BOUND_CONSTANT = 10
    def __init__(self):
        super().__init__('navigator_car')
        self.bReceivedPath = False
        self.bNoiseChar = False
        self.bFirstPath = True
        self.bSolution = False
        self.noise_time = 2
        self.avg_vel = .3 
        self.path_index = -1



        self.path_subscriber = self.create_subscription(Path,
                                                '/path_planner/path',
                                                self.path_listener,
                                                10)
        self.solution_subscriber = self.create_subscription(Bool,
                                                '/localization/solution_found',
                                                self.solution_listener,
                                                1)
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
        
        self.waypoint_publisher = self.create_publisher(Float32,
                                                   '/controller/waypoint_heading', 
                                                   1)
        
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=1
                )
        
        self.cmd_publisher = self.create_publisher(Float32MultiArray,
                                                   '/controller/cmd', 
                                                   qos_profile=qos_profile)

        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                depth=5
                )
        self.next_waypoint_publisher = self.create_publisher(PoseStamped,
                                                   '/controller/next_waypoint', 
                                                   1)
        #self.timer = self.create_timer(timer_period, self.update_cmd)
    #The frame is severely messed up for some reason need to come back and fix...
       
    def flip_input(self, position, orientation):
        pos = np.array([position.x, position.y, position.z]).transpose()
        new_pos = np.matmul(self.aFlip.as_matrix(), pos) 
        tmpr = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

        newR = R.from_matrix(np.matmul(self.aFlip.as_matrix(),tmpr.as_matrix()))
        return new_pos, newR.as_quat() 

    def pose_listener(self, msg):
        #Basically just save this so we always have an estimate of this state
        self.current_state = msg.pose
        if self.bReceivedPath == True and self.bNoiseChar == True:
            #Check if we've hit the waypoint
            tcar_ref = self.current_state.position
            tway_ref = self.next_pose.position
            car_q = self.current_state.orientation
            way_q = self.next_pose.orientation
            #Convert into np arrays
            tcar_ref = np.array([tcar_ref.x, tcar_ref.y, tcar_ref.z])
            tway_ref = np.array([tway_ref.x, tway_ref.y, tway_ref.z])
            car_q = np.array([car_q.x, car_q.y, car_q.z, car_q.w])
            way_q = np.array([way_q.x, way_q.y, way_q.z, way_q.w])
            #Convert to the rot obj
            rcar = R.from_quat(car_q)
            rway = R.from_quat(way_q)
            #Get out DCMs
            Rref_car = rcar.as_matrix()
            Rref_way = rway.as_matrix()
            

            self.get_logger().info('LOCATION---------------------')
            self.get_logger().info('Current Position: <%.4f, %.4f, %.4f> rotation: <%.4f, %.4f, %.4f, %.4f>' %
                (tcar_ref[0], tcar_ref[1], tcar_ref[2], car_q[0], car_q[1], car_q[2], car_q[3]))
            self.get_logger().info('Next Waypoint: <%.4f, %.4f, %.4f> rotation: <%.4f, %.4f, %.4f, %.4f>' %
                (tway_ref[0], tway_ref[1], tway_ref[2], way_q[0], way_q[1], way_q[2], way_q[3]))

            #assess differences in reference frame
            dx = tway_ref[0] - tcar_ref[0]
            dy = tway_ref[1] - tcar_ref[1]

            #Translate the waypoint into the car frame
            way_car = np.matmul(Rref_car.transpose(), tway_ref - tcar_ref)
            #Reconstruct the angle
            opp_length = way_car[1]
            adj_length = way_car[0]
            waypoint_heading = np.arctan(opp_length/adj_length)
            waypoint_heading_float = Float32()
            waypoint_heading_float.data = waypoint_heading
            self.waypoint_publisher.publish(waypoint_heading_float)
            #other things, need to make sure the motion along the path works
            # and then also, make sure to change the constant 4 value below back to 1 or 2 
            cmd_angle = waypoint_heading
            self.get_logger().info('CAR FRAME LOCATION ----------------')
            self.get_logger().info('Waypoint: <%.4f, %.4f, %.4f>' % (way_car[0], way_car[1], way_car[2]))
            self.get_logger().info('Waypoint heading: %.4f degrees' % np.rad2deg(waypoint_heading))
    
            #If you miss the rotation, hopefully it's not that bad.... but you can't really do a complete 180 in place or wahtever
            if abs(dx) < self.x_limit and abs(dy) < self.y_limit:# and abs(dtheta) < self.theta_limit:
                self.get_logger().info('HOOOOOOORAY!   Achieved Waypoint with the following stats')
                
                #if self.path_index >= len(self.path.poses):
                if self.path_index < 0:
                    self.avg_vel = 0.0
                    self.get_logger().info('out of poses in the path, are at the end?')
                    
                else:
                    self.next_pose = self.path.poses[self.path_index].pose
                    self.next_spose = self.path.poses[self.path_index]
                    self.path_index-=1
            #elif abs(np.rad2deg(waypoint_heading)) > 90:
            elif adj_length <= 0:
                #This is the indicator we missed it... so move on and forget
                self.get_logger().info('BOOOOOO Waypoing missed')
                #if self.path_index >= len(self.path.poses):
                if self.path_index < 0:
                    self.avg_vel = 0.0
                    self.get_logger().info('Out of poses, sort of at the end')
                else:
                    self.next_pose = self.path.poses[self.path_index].pose
                    self.next_spose = self.path.poses[self.path_index]
                    self.path_index-=1
        

            #update the command            
            if self.bSolution == True:
                #Always wat to set this to 0
                cmd_theta = 0.0
                cmd_vel = self.avg_vel
            else:
                self.get_logger().info('No solution, velocity and rotation set to 0')
                #if we lose track and are waiting
                cmd_theta = 0.0
                cmd_vel = 0.0

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
            #Also publish the next waypoint
            self.next_waypoint_publisher.publish(self.next_spose)
                
    def path_listener(self, msg):
        self.path = msg
        #do this to let some time pass allowing for better noise characteirzation
        #if self.bFirstPath == True:
        #    self.prev_time = datetime.now() 
        #    self.bFirstPath = False

        #duration = (prev_time - datetime.now()).total_seconds() 

        #if self.bNoiseChar == False and duration > self.noise_time
        
        
        if self.bNoiseChar == False:
            #self.get_logger().info('Sleeping for 4 seconds before ingesting path')
            #sleep(4.0)        
            self.path_index = len(self.path.poses)-1
            start_pose = self.path.poses[self.path_index].pose
            self.path_index-=2
            #Try to understand the deviations from the current state (as ideally
            # they should be near identical. Can characterize the noise this way)
            dx = self.current_state.position.x - start_pose.position.x
            dy = self.current_state.position.y - start_pose.position.y
            qstart = self.current_state.orientation            
            theta_start = self.quat_to_theta(qstart)
            qnow = start_pose.orientation            
            theta_now = self.quat_to_theta(qnow)
            #self.get_logger().info('Theta Now %.4f' % theta_now)
            #self.get_logger().info('Theta Start Path %.4f' % theta_now)

            self.get_logger().info('Current State Pos %.4f, vs. path start %.4f' % (self.current_state.position.x, start_pose.position.x))
            dtheta = theta_now - theta_start

            #define the noise values
            self.theta_limit = self.BOUND_CONSTANT * abs(dtheta)
            self.x_limit = self.BOUND_CONSTANT * abs(dx)
            self.y_limit = self.BOUND_CONSTANT * abs(dy)
            self.start_pose = start_pose
            self.get_logger().info('Path length identified as %d, indexing at %d' % (len(self.path.poses),self.path_index))


            #Hand-tuned noise values
            self.theta_limit = np.deg2rad(15)
            self.x_limit = .1
            self.y_limit = .1
            self.get_logger().info('Bounds as <%.4f, %.4f> %.4f deg' % (self.x_limit, self.y_limit, np.rad2deg(self.theta_limit)))
            self.next_pose = self.path.poses[self.path_index].pose
            self.next_spose = self.path.poses[self.path_index]
            #Publish the next pose
            self.next_waypoint_publisher.publish(self.next_spose)
            
            #self.projected_time = dx/self.avg_vel
            
            self.path_index-=1
            self.bNoiseChar = True
        
        #deduce the next waypoint as ideally the next one in the chain...        
        self.bReceivedPath = True
    def solution_listener(self, msg):
        self.bSolution = msg.data

    def quat_to_thetav2(self, q):
        r = R.from_quat(q)
        e = r.as_euler('zyx')
        return e[0]
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
