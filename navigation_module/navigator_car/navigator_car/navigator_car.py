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
            cur_pos = self.current_state.position
            way_pos = self.next_pose.position
            cur_or = self.current_state.orientation
            next_or = self.next_pose.orientation
            #self.aFlip = R.from_euler('z', np.pi/2)

            #Flip into this frame (hopefully)
            #cur_pos, cur_or = self.flip_input(cur_pos, self.current_state.orientation)
            #way_pos, next_or = self.flip_input(way_pos, self.next_pose.orientation)
            cur_pos = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
            way_pos = np.array([way_pos.x, way_pos.y, way_pos.z])
            cur_or = np.array([cur_or.x, cur_or.y, cur_or.z, cur_or.w])
            next_or = np.array([next_or.x, next_or.y, next_or.z, next_or.w])
            cur_theta = self.quat_to_thetav2(cur_or)
            way_theta = self.quat_to_thetav2(next_or)

            self.get_logger().info('LOCATION---------------------')
            self.get_logger().info('Current Position: <%.4f, %.4f, %.4f> rotation: <%.4f, %.4f, %.4f, %.4f>' %
                (cur_pos[0], cur_pos[1], cur_pos[2], cur_or[0], cur_or[1], cur_or[2], cur_or[3]))
            self.get_logger().info('Current rotation angle: %.4f' % np.rad2deg(cur_theta))
            self.get_logger().info('Next Waypoint: <%.4f, %.4f, %.4f> rotation: <%.4f, %.4f, %.4f, %.4f>' %
                (way_pos[0], way_pos[1], way_pos[2], next_or[0], next_or[1], next_or[2], next_or[3]))
            self.get_logger().info('Next waypoints theta %.4f' % np.rad2deg(way_theta))


            r = R.from_quat(next_or) 

            way_car = np.matmul(r.as_matrix().transpose(), way_pos)
            cur_car= np.matmul(r.as_matrix().transpose(), cur_pos)
            dx_car = way_car[0] - cur_car[0]
            dy_car = way_car[1] - cur_car[1]
            #X points straight out (I think), so redefine this
            waypoint_heading_car = np.arctan(dy_car/dx_car)
            self.get_logger().info('CAR FRAME')
            self.get_logger().info('way car: <%.4f, %.4f> cur car <%.4f, %.4f>, diff = <%.4f %.4f' % 
                (way_car[0], way_car[1], cur_car[0], cur_car[1], dx_car, dy_car))
            self.get_logger().info('theta = %.4f' % np.rad2deg(waypoint_heading_car))
            #cur_theta = self.quat_to_theta(self.current_state.orientation)
            #way_theta = self.quat_to_theta(self.next_pose.orientation)
            #qway  = self.next_pose.orientation
            #qway  = next_or
            #r = R.from_quat([qway.x, qway.y, qway.z, qway.w])
            #dx = (cur_pos.x - way_pos.x)
            #dx = (way_pos.x - cur_pos.x)
            dx = way_pos[0] - cur_pos[0]
            #dy = (cur_pos.y - way_pos.y)
            #dy = (way_pos.y - cur_pos.y)
            dy = way_pos[1] - cur_pos[1]
            #vect_car = -1.0*np.matmul(np.matmul(aFlip.as_matrix(), r.as_matrix().transpose()), np.array([dx, dy, 0]))
             
            vect_car = np.matmul(r.as_matrix().transpose(), np.array([dx, dy, 0]))
            waypoint_car = np.matmul(r.as_matrix().transpose(), np.array([way_pos[0], way_pos[1], 0]))
            self.get_logger().info('Waypoint in car frame: <%.4f %.4f>' % (waypoint_car[0], waypoint_car[1]))
            self.get_logger().info('Vector in world frame: <%.4f, %.4f> - vector in car frame: <%.4f, %.4f>' %
                (dx, dy, vect_car[0], vect_car[1]))
            x_unit = np.array([1,0 ,0])
            waypoint_heading_angle = np.arccos(np.dot(vect_car, x_unit)/(np.linalg.norm(x_unit) * np.linalg.norm(vect_car)))
            waypoint_sign = np.cross(vect_car, x_unit)
            otherwaypoint_sign = np.cross(vect_car, -x_unit)
            waypoint_sign = waypoint_sign[-1]
            otherwaypoint_sign = otherwaypoint_sign[-1]
            #Better way to get the angle
            waypoint_heading_angle = np.arctan(dx/dy)
            dtheta = (cur_theta - way_theta)
            off_heading = waypoint_heading_angle - cur_theta
            #I think this works... trying to teest but the car shit out on me and so did
            # the path planner. Will have to try again tomorrw, the testing is slow going
            # Basically want to make sure that positive is for left and negative is for right
            # otherwise will need to flip it
            #other things, need to make sure the motion along the path works
            # and then also, make sure to change the constant 4 value below back to 1 or 2 
            cmd_angle = waypoint_heading_car
    
            self.get_logger().info('Waypoint Heading = %.4f, direction %.4f, otherdir %.4f, difference is %.4f' % (np.rad2deg(waypoint_heading_angle), waypoint_sign, otherwaypoint_sign, np.rad2deg(off_heading)))
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
            elif np.rad2deg(waypoint_heading_angle) > 90:
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
        
            #self.get_logger().info('Current pos <%.4f, %.4f> %.4f deg vs. Waypoint pos <%.4f, %.4f> %.4f deg' 
            #                        % (cur_pos[0], cur_pos[1], np.rad2deg(cur_theta), way_pos[0], way_pos[1], np.rad2deg(way_theta)))
                               #     % (cur_pos.x, cur_pos.y, cur_theta,
                               #         way_pos.x, way_pos.y, way_theta))
            #self.get_logger().info('Deltas <%.4f, %.4f> %.4f deg' %
            #                        (dx, dy, np.rad2deg(dtheta)))

            #update the command            
            if self.bSolution == True:
                #cmd_theta = way_theta
                cmd_theta = cmd_angle
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
            self.path_index-=5
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
