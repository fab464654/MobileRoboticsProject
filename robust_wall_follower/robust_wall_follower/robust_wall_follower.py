"""
            +----------------------------------------------+
            |           ROBUST WALL FOLLOWER               |
            |   WITH PRECISE WALL ALIGNMENT USING RANSAC   |
            +----------------------------------------------+

                Mobile Robotics Project a.y. 2021/2022

                            Authors:
           Castellini Fabio, Sandrini Michele (June/July 2022)
"""

import rclpy
from rclpy.node import Node
import rclpy.qos
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import numpy as np
import math
import time
from pynput import keyboard
from collections import deque
import copy
import sys
import os
from warnings import warn
from .functions.ransac import *
from .functions.auxiliary_functions import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# START Turtlebot3RobustWallFollower =====================================================================================================================
class Turtlebot3RobustWallFollower(Node):

    def __init__(self, distance_th=0.15,front_angle_half=90,focus_angle_half=30,side_angle=20,ransac_th=0.01, ransac_iter=100, add_noise=False, sigma=0.001, K_P=1, align_max_ang_vel=0.5):
        super().__init__('turtlebot3_RobustWallFollower_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        # Subscription to odometry topic, to get the estimated current Pose of the robot
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, rclpy.qos.qos_profile_sensor_data)
        # Publisher on topic '/ransac_lidar_view', to display in RViz the subset of lidar points, used
        # to estimate the best regressor line with RANSAC (green points in RViz2)
        self.ransac_lidar_view_pub = self.create_publisher(LaserScan, '/ransac_lidar_view', 1)
        # Publisher on topic '/current_state', where a simple String representing the current state
        # of the FSM (Finite State Machine) is published, in order to show it in RViz2
        self.current_state_str_pub = self.create_publisher(Marker, '/current_state', 1)
        # Publisher to topic '/line_visualization', where a Marker LINE_STRIP message allows
        # to represent the estimated regressor line on RViz2 (in yellow)
        self.line_visualization_pub = self.create_publisher(Marker, '/line_visualization', 1)
        # Publisher for representing in RViz2 the path followed by the Turtlebot3
        self.path_pub = self.create_publisher(Path, '/path', 1)
        # Publisher on topic '/front', '/left' and '/right' the subset of lidar points from the relative
        # regions, to which the noise could have been added
        self.front_pub = self.create_publisher(LaserScan, '/front', 1)
        self.narrow_front_pub = self.create_publisher(LaserScan, '/narrow_front', 1)
        self.left_pub = self.create_publisher(LaserScan, '/left', 1)
        self.right_pub = self.create_publisher(LaserScan, '/right', 1)
        self.focus_view_pub = self.create_publisher(LaserScan, '/focus_view', 1)

        
        # Initialize keyboard Listener to detect keys inserted by user
        keyboard.Listener(on_press=self.press_callback).start()
        self.key_pressed = False # this boolean variable is set to True when user insert any key

        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
        self.regions = {
            'front': 0.0,       
            'narrow_front': 0.0,   # movable sector of lidar, from which input points for RANSAC are extracted
            'left': 0.0,
            'right': 0.0,
        }

        # Initialization of dict storing the indeces of the min lidar readings in the relative region
        self.argmin_regions = {
            'front': 0,
            'narrow_front': 0,
        }

        # definition of dict with state of FSM
        self.state_dict_ = {
            0: 'find the wall',
            1: 'align left',
            2: 'follow the wall',
            3: 'align right',
            4: 'rotate 180 deg',
            5: 'rewind',
        }
        # velocity command
        self.msg = Twist()

        # Current lidar readings
        self.scan = LaserScan()
        self.original_scan = LaserScan()

        # distance self.threshold to the wall
        self.th = distance_th # [m] # in simulation 0.15 m, in real robot 0.25

        timer_period = 0.1  # [seconds]

        # Initializations of:
        self.twist = Twist() # current robot twist (linear+angular velocity), from /odom topic
        self.theta = 0.0 # current orientation of the robot, extracted from '/odom' topic 
        self.init_theta = None # initial theta. Used to make the robot rotating exactly of 180°: (current orientation - self.init_theta) must be 180°
        self.rot180deg_end = False # boolean var. to specify when the rotation of 180° of the robot has ended
        self.rot180_to_rewind = True # represents the state before "rotate 180°" or equivalently, the state before pressing a button (see further explanation at the end of this file, section "EXPLANATIONS", point 1) )
        

        # Data structure to implement a STACK, in which the each velocity message
        # is saved/stored. These will be used in 'rewind' state to repeat past actions
        self.vel_stack = deque(maxlen=1800) # stack saves velocities for at maximum 3 minutes (see below) (just a choice)
        # timer_period = 0.1 sec -> 10 cycles/sec -> 600 cycles/min
        # So, for instance, in 3 min -> 1800 cycles

        # Fields added for "robust_wall_follower"

        # Array of (current) lidar readings
        self.ranges = []

        # Starting index, where the minimum distance in self.ranges is detected 
        self.index_init = None
        self.wall_dir_idx_in_lidar = None

        # Initial angle, when robot enters in "align left/right" state
        self.angle_init = None

        # Frontal region used in 'find wall' state
        self.front_angle_half = front_angle_half

        # The angle width of lateral regions 'left' and 'right' (in degrees/indeces)
        # 'left' = [self.front_angle_half, self.front_angle_half+self.side_angle]
        # 'right' = [-self.front_angle_half-self.side_angle, -self.front_angle_half]
        self.side_angle = side_angle

        # Half width of region used to extract lidar points used in RANSAC, during 'align left/right' or 'follow the wall' states
        self.focus_angle_half = focus_angle_half # 30

        # Maximum saturated angular velocity reachable in 'align right/left' state
        self.align_max_ang_vel = align_max_ang_vel

        # Boolean variable. If true, white noise is added to lidar readings (for a more realistic simulation)
        self.add_noise = add_noise
        self.sigma = sigma # standard deviation of the white noise (mean 0)

        # Wall line linear coefficient, to which the robot should align
        self.m = None
        self.wall_line = RansacLine(ransac_th,ransac_iter,add_noise=False) # noise added directly in lidar readings

        # Controller Proportional term
        self.K_P = K_P
        # Desired angular coefficient of the Line that has to be commanded thanks to controller
        self.m_d = 0

        # Semaphore to indicate when "alignment" is complete
        self.align_end = False

        # Variable in which the path followed by the Turtlebot3 is stored and then visualized in RViz2
        self.path = Path()

        # Ros parameter (name, default_value)
        self.declare_parameter('align_direction','left')

        param = self.get_parameter('align_direction').get_parameter_value().string_value

        if param=='left' or param=='right':
            self.turn_left = param=='left'
        else:
            raise TypeError("Only parameters accepted are 'left' or 'right'!")

        self.timer = self.create_timer(timer_period, self.control_loop)
    # END __init__()
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # Method for listening to the keyboard
    def press_callback(self,key): # PAY ATTENTION TO THE PARAMETERS ORDER!!!
        if key==keyboard.KeyCode.from_char('r'):
            if self.state_!=4:
                self.key_pressed = True # always read from keyboard, except for when
                                        # the robot is rotating of 180° (self.state_==4,
                                        # 'rotate 180 deg'), This is an "interruptible"
                                        # routine (at least I designed it like that)
            else:
                print("<keyboard listener>: ATTENTION: <rotate 180 deg> is an interruptible routine")
                print("                     (keyboard listener restarts after it finishes)\n")
        else:
            print("<keyboard listener>: only allowed char is 'r', to start/stop <rewind> procedure!\n")
    
    # To read from robot odometry the actual current robot orientation (self.theta)
    def odom_callback(self,msg):
        # Quaternion expressing the orientation
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        rpy = quat2eul(x,y,z,w) # conversion quaternion -> Euler angles (2nd angle is the only interesting one for us)

        self.theta = rpy[0]%(2*math.pi) # self.theta in range [0,2*pi) [rad]!

        # Append current Pose to self.path variable, to visualize the path in RViz2
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
    # END press_callback()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # loop each 0.1 seconds
    def control_loop(self):

        # Check whether the previous velocity command send to the robot has been executed correctly. If yes, save it
        """save_vel = abs(self.msg.linear.x - self.lin_vel) < 0.01
        print("abs(self.msg.linear.x - self.lin_vel) ", abs(self.msg.linear.x - self.lin_vel), "< 0.01")
        save_vel = save_vel and abs(self.msg.angular.z - self.ang_vel) < 0.01
        print("abs(self.msg.angular.z - self.ang_vel) ", abs(self.msg.angular.z - self.ang_vel), "< 0.01", end="\n\n")"""
    
        # callback actions associated to each state
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
        elif self.state_ == 3:
            self.align_right()
        elif self.state_==4 or (self.state_==5 and len(self.vel_stack)==0): # rotate also if the current state is "rewind" and the stack was emptied
            self.rotate180deg()
        elif self.state_==5:
            self.rewind()
        else:
            print('Unknown state!')
        
        # Upload/save velocities commands, only if not in state 'rotate 180 deg' or 'rewind'
        if self.state_ in [0,1,2,3]:
            #print("Saving velocity in stack. len: ", len(self.vel_stack))
            current_msg = copy.deepcopy(self.msg) # necessary otherwise stack contains addressess
                                                    # of THE same single variablE self.msg => all with the same values!
            self.vel_stack.append(current_msg)
        
        # To publish a string representing the current state to topic '/current_state' subscripted by Rviz
        self.publish_current_state_marker()

        if self.state_ in [1,2,3]:
            self.pub_line_marker()
        
        # If necessary, saturate velocities if they exceed limits
        if abs(self.msg.linear.x)>1: # m/s
            self.msg.linear.x = 1 * np.sign(self.msg.linear.x)
            warn("ATTENTION: linear velocity along x, {:.2f} m/s, was too high so it has been saturated to 1 m/s!")
        if abs(self.msg.angular.z)>1.82: #rad/s
            self.msg.angular.z = 1.82 * np.sign(self.msg.angular.z)
            warn("ATTENTION: angular velocity around z, {:.2f} rad/s, was too high so it has been saturated to 1.82 m/s!")

        self.publisher_.publish(self.msg)

        # FOR DEBUGGING:
        # 0: 'find the wall', 1: 'align left', 2: 'follow wall', 3: 'align right', 4: 'rotate 180 deg', 5: 'rewind'
        print("State: <{:s}>".format(self.state_dict_[self.state_]) )
        print("    Lidar (min): front[{:4d},{:4d}]: {:>5.3g} [m] -> argmin: {:3d}, focus_view[{:>4s},{:>4s}]: {:s} [m] -> argmin: {:>3s}".format(
            -self.front_angle_half, self.front_angle_half, self.regions['front'], self.argmin_regions['front'],
            str(self.wall_dir_idx_in_lidar-self.focus_angle_half) if self.state_ in [1,2,3] else str(None),
            str(self.wall_dir_idx_in_lidar+self.focus_angle_half) if self.state_ in [1,2,3] else str(None),
            str(round(self.regions['narrow_front'],3)) if self.state_ in [1,2,3] else str(None),
            str(int(self.argmin_regions['narrow_front'])) if self.state_ in [1,2,3] else str(None) ) )
        print("    Left[{:4d},{:4d}]: {:>6.3g} [m], Right[{:4d},{:4d}]: {:>6.3g} [m].      Odometry: orientation: {:>3d}° (={:>7.5g} rad)".format(
            self.front_angle_half, self.front_angle_half+self.side_angle, self.regions['left'],
            len(self.ranges)-self.front_angle_half-1-self.side_angle, len(self.ranges)-self.front_angle_half-1, self.regions['right'],
            int(np.around(np.degrees(self.theta))), self.theta ) )
        print("    Velocity (Twist): lin. v.: {:>5.2g} [m/s], ang. v.: {:>5.2g} [rad/s] , num. vel. msgs saved in STACK: {:4d}/{:d}".format(self.msg.linear.x, self.msg.angular.z, len(self.vel_stack), self.vel_stack.maxlen ) )
        print("    Line angular coefficient m: {:>8s},   wall_direction: {:>3s},  len(original_lidar): {:4d}, len(self.ranges): {:4d}".format(
            str(round(self.wall_line.m,5)) if isinstance(self.wall_line.m, float) else str(self.wall_line.m), str(self.wall_dir_idx_in_lidar), len(self.original_scan.ranges.tolist()), len(self.ranges) ), end="\n\n" )

    # END control_loop()
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def reset_focus_view(self):
        self.index_init = None
        self.wall_dir_idx_in_lidar = None
        self.regions['narrow_front'] = None
        self.argmin_regions['focus view'] = None
        self.wall_line.reset()
        self.pub_line_marker()

        self.pub_ransac_lidar_indeces(None,self.wall_line.lidarInputPoints) # publish "empty" msg
    # END reset_focus_view()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def publish_current_state_marker(self):
        msg = Marker()
        msg.header.frame_id = "/base_footprint"
        msg.ns = ""
        msg.id = 0
        msg.type = msg.TEXT_VIEW_FACING
        msg.action = msg.ADD

        msg.pose.position.x = 0.15*math.sin(self.theta)
        msg.pose.position.y = 0.15*math.cos(self.theta)
        msg.pose.position.z = 0.5
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.text = self.state_dict_[self.state_].replace(" ", "_")

        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1

        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 0.5
        msg.color.a = 1.0
        self.current_state_str_pub.publish(msg)
    # END publish_current_state_marker()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            
    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 

        self.original_scan = msg

        self.scan = msg
        if self.add_noise:
            self.scan.ranges = [self.scan.ranges[i]+np.random.normal(0,self.sigma) for i in range(len(self.scan.ranges))]

        # Fundamental step in real Turtlebot3: reconstruct the lidar readings array, such that its length is 360
        # (See at the end of this file, paragraph "EXPLANATIONS", point ) for further details)
        self.scan = add_lidar_readings(self.scan)

        self.ranges = self.scan.ranges.tolist()

        # Initialization of the three LaserScan() messages relative to the lidar
        # readings (point cloud) relative respectively to region 'front', 'left' and 'right'
        length = len(self.ranges)
        front_msg = copy.deepcopy(self.scan)
        narrow_front_msg = copy.deepcopy(self.scan)
        left_msg = copy.deepcopy(self.scan)
        right_msg = copy.deepcopy(self.scan)

        for i in range(length):
            # Front region
            if not i in range(self.front_angle_half) and not i in range(length-self.front_angle_half, length):
                front_msg.ranges[i] = np.nan
            # else: keep the value unchanged

            # Narrow front region
            if not i in range(-self.focus_angle_half) and not i in range(length-self.focus_angle_half, length):
                narrow_front_msg.ranges[i] = np.nan
            # else: keep the value unchanged

            # Left region
            if not i in range(self.front_angle_half, self.front_angle_half+self.side_angle):
                left_msg.ranges[i] = np.nan
            # else: keep the value unchanged

            # Right region
            if not i in range(length-self.front_angle_half-1-self.side_angle, length-self.front_angle_half-1):
                right_msg.ranges[i] = np.nan
            # else: keep the value unchanged


        # Publish the LaserScan() messages relative to 'front', 'narrow_front', 'left' and 'right'
        self.front_pub.publish(front_msg)
        self.narrow_front_pub.publish(narrow_front_msg)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        
        # Update regions according to lidar readings
        self.regions['front'] = np.nanmin(front_msg.ranges)
        self.regions['narrow_front'] = np.nanmin(narrow_front_msg.ranges)
        self.regions['left'] = np.nanmin(left_msg.ranges)
        self.regions['right'] =  np.nanmin(right_msg.ranges)

        # Returns the index relative to minimum stored in "self.regions['front']"
        self.argmin_regions['front'] = np.nanargmin(front_msg.ranges)

        # Returns the index relative to minimum stored in "self.regions['narrow_front']"
        self.argmin_regions['narrow_front'] = np.nanargmin(narrow_front_msg.ranges)

        # function where are definied the rules for the change state
        self.take_action()
        
    # END laser_callback()
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def pub_ransac_lidar_indeces(self, idxs, lidar_dists):

        # Publish noisy / normal lidar ranges
        msg = self.scan

        if lidar_dists!=None:
            # Initialize all lidar values to 0.0
            msg.ranges = [0.0 for _ in range(len(msg.ranges))]
            msg.intensities = [0.0 for _ in range(len(msg.ranges))]
            
            # Overwrite lidar distances only in given indeces "idxs", which represent
            # the region of interest where the wall is, and whose points are the input
            # of ransac algorithm, that computes the relative line angular coefficient
            for i,index in enumerate(idxs):
                msg.ranges[index] = lidar_dists[i]
            #for i in range(len(msg.ranges)):
            #    if not i in idxs:
            #        msg.ranges[i] = 0.0
        else:
            # Delete the regione where ransac points are extracted and showed in Rviz
            for i in range(len(msg.ranges)):
                msg.ranges[i] = 0.0
                msg.intensities[i] = 0.0

        self.ransac_lidar_view_pub.publish(msg)



    # END pub_ransac_lidar_indeces()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # Method to publish Line Marker to topic '/line_visualization', to visualize the resulting
    # line extracted from RANSAC in Rviz2
    def pub_line_marker(self):
        
        # To print a line in RViz2, the visualization_msgs.msg Marker must be used.
        # In particular, to draw a line (a segment actually), are necessary the two
        # extremants points.
        # To find these points, we compute the INTERSECTION BETWEEN the LINE (equation)
        # and a CIRCLE of a certain radius R...

        # Initialization of the Marker, LINE_STRIP, message...
        marker = Marker()
        marker.header.frame_id = "/base_footprint"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.025
        marker.scale.y = 0.025
        marker.scale.z = 0.025

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.27
        marker.color.b = 0.0

        # marker orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # If we are in a state of the FSM (Finite State Machine) where ransac is used,
        # then only in these cases the line should be drawn.
        # Otherwise, when RANSAC is not used, the line coefficient is None
        # (self.wall_line.m) ...
        if self.wall_line.m!=None:
            # Extremants points through which the line, or better the segment pass
            # Extremants points through which the line, or better the segment pass
            m = self.wall_line.m
            q = self.wall_line.q
            R = self.th + 0.13
            delta = (m**2 * q**2) - (1 + m**2)*(q**2 - R**2)
            
            counter=0
            while delta<=0 and counter<=100:
                counter+=1
                R+=0.05
                delta = (m**2 * q**2) - (1 + m**2)*(q**2 - R**2)
                print("{}. delta: {}".format(counter, delta) )

            if delta<=0:
                raise Exception("Error , delta is <=0")
            
            x1 = (-m*q + math.sqrt(delta))/(1 + m**2)
            x2 = (-m*q - math.sqrt(delta))/(1 + m**2)

            y1 = m*x1 + q
            y2 = m*x2 + q
        else:
            # When line angular coefficient is None, then the line should not be displayed.
            # To do that, the two extreme points are collapsed both to (0,0), such that no
            # line is visible

            x1 , y1 = 0.0, 0.0
            x2 , y2 = 0.0, 0.0

        # marker line points
        marker.points = []

        # first point
        first_line_point = Point()
        first_line_point.x = x1
        first_line_point.y = y1
        first_line_point.z = 0.0
        marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = x2
        second_line_point.y = y2
        second_line_point.z = 0.0
        marker.points.append(second_line_point)

        # Publish the Marker
        self.line_visualization_pub.publish(marker)


    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # Method to publish Line Marker to topic '/line_visualization', to visualize the resulting
    # line extracted from RANSAC in Rviz2
    def pub_turtlebot_path(self):
        pass
        

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def take_action(self):
        # This method implements the EDGES of the FSM (Finite States Machine)

        # 0: 'find the wall', 1: 'align left', 2: 'follow wall', 3: 'align right', 4: 'rotate 180 deg', 5: 'rewind'

        # '*all states except 4 itself*' --> 'rotate 180 deg'
        if (self.state_ in [0,1,2,3,5] and self.key_pressed) or (self.state_==5 and len(self.vel_stack)==0):  # exit from 'rewind' if user has pressed a key or velocities history is empty
            self.key_pressed = False # reset value for next time
            if len(self.vel_stack)==0:
                print("Velocities history empty! Resuming 'find the wall' task...")
            self.change_state(4)
        
        # 'rotate 180 deg' ---> 'rewind'
        elif self.state_==4 and self.rot180deg_end and self.rot180_to_rewind:
            self.rot180deg_end = False
            self.rot180_to_rewind = False
            self.change_state(5) # E4
        
        # 'rotate 180 deg' ---> 'find wall'
        elif self.state_==4 and self.rot180deg_end and not self.rot180_to_rewind:
            self.rot180deg_end = False
            self.rot180_to_rewind = True
            self.change_state(0) # E5

        # 'find wall' --> 'align left'
        elif self.state_==0 and self.regions['front']<self.th and self.turn_left:
            self.change_state(1)
        
        # 'follow the wall' --> 'align left'
        elif self.state_==2 and self.regions['narrow_front']<self.th and self.turn_left:
            self.change_state(1)
        
        # 'find wall' --> 'align right'
        elif self.state_==0 and self.regions['front']<self.th and not self.turn_left:
            self.change_state(3)
        
        # 'follow the wall' --> 'align right'
        elif self.state_==2 and self.regions['narrow_front']<self.th and not self.turn_left:
            self.change_state(3)
        
        # 'follow wall' --> 'find the wall'
        elif self.state_==2 and ( (self.turn_left and self.regions['right']>self.th) or (not self.turn_left and self.regions['left']>self.th) ):
            self.change_state(0)

        # 'find_wall' OR 'align_left' --> 'follow the wall'
        elif self.state_==0 and self.regions['narrow_front']>self.th and self.align_end and self.turn_left:
            self.change_state(2)
        
        # 'align left' --> 'follow the wall'
        elif self.state_==1 and self.align_end and self.turn_left:
            self.change_state(2)
        
        # 'find_wall' OR 'align_right' --> 'follow the wall'
        elif self.state_==0 and self.regions['narrow_front']>self.th and self.align_end and not self.turn_left:
            self.change_state(2)
        
        # 'align_right' --> 'follow wall'
        elif self.state_==3 and self.align_end and not self.turn_left:
            self.change_state(2)

        else:
            pass # FSM remains in the same state
    # END take_action()

        
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # function to update state
    # don't modify the function
    def change_state(self, state):

        previous_state = self.state_
        
        if state is not self.state_:

            # When 'align left' or 'align right' are finished, reset variable 'self.align_end'
            if self.state_ in [1,3]:
                self.align_end = False

            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]), end="\n\n")
            self.state_ = state

            # if you are changing state into 'align left/right', then initialize variables...
            if state in [1,3]: # align left / align right
                if previous_state==0: # if previous_state was 'find wall'...
                    self.index_init = self.argmin_regions['front'] # ...use the wall found in the wide frontal area
                else:
                    self.index_init = self.argmin_regions['narrow_front'] # ...otherwise use the narrower section 'narrow_front'
                self.angle_init = self.theta

            # 0: 'find the wall', 1: 'align left', 2: 'follow wall', 3: 'align right', 4: 'rotate 180 deg', 5: 'rewind'
            elif state==0 or state==4:
                self.reset_focus_view()
        else:
            print("", end="\n\n") # just for visualization
    # END change_state()

    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # action to find the wall, move forward and wall side to find the wall
    def find_wall(self):

        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.08 #m/s

        if self.turn_left:
            self.msg.angular.z = -0.405 #rad/s (SUGGESTED: -0.3 in real burger robot, -0.5 in simul)
        else:
            self.msg.angular.z = 0.43 #rad/s (SUGGESTED: 0.3 in real burger robot, 0.5 in simul)
    # END find_wall()
        
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    # action to torn left, move forward and left side
    def align_left(self):
        delta_theta = round(np.rad2deg(self.theta - self.angle_init))

        # The direction in the current robot coordinate frame, toward which the wall is
        self.wall_dir_idx_in_lidar = int((360 + (self.index_init - delta_theta)) % 360)
        
        # Around the "self.wall_dir_idx_in_lidar"
        # [self.wall_dir_idx_in_lidar - self.focus_angle_half, self.wall_dir_idx_in_lidar + self.focus_angle_half]
        limit1 = self.wall_dir_idx_in_lidar - self.focus_angle_half
        limit2 = self.wall_dir_idx_in_lidar + self.focus_angle_half

        minIndex = min(limit1, limit2)
        maxIndex = max(limit1, limit2)

        indeces = list(range(minIndex,maxIndex+1))
        indeces = orderLaserScanSectionIndeces(indeces)

        
        # Run RANSAC algorithm
        pointsForRansac = np.array(self.ranges)[indeces]
        anglesForRansac = np.array(indeces)

        # To test WITH noise
        #inliers_1, outliers_1, self.m, q_best_1, lidar_measurements = ransac(self.threshold, self.iterations, pointsForRansac, anglesForRansac, self.add_noise, self.sigma)
        self.wall_line.find_with_ransac(pointsForRansac,anglesForRansac)

        # Send to topic and to Rviz, the LaserPoints of the lidar section used for RANSAC
        self.pub_ransac_lidar_indeces(indeces, self.wall_line.lidarInputPoints)

        # Use proportional controller only when 'self.wall_dir_idx_in_lidar' is in range [200,340]
        # (this means wall is on robot right). Otherwise use a constant counter-clockwise ang. vel.
        # until wall ('self.wall_dir_idx_in_lidar'), is on the right (and P control is finally used)
        # (For further explanations see at the bottom of this file, "EXPLANATIONS", point 2) )
        if self.wall_dir_idx_in_lidar in range(200,340+1):
            # Angular coefficient Error: desired m - current m
            err_m = self.m_d - self.wall_line.m

            if abs(err_m)>0.01:
                self.msg.linear.x = 0.0 #m/s
                self.msg.angular.z = - self.K_P * err_m
            
            else:
                self.msg.linear.x = 0.0 #m/s
                self.msg.angular.z = 0.0 #rad/s
                # change state
                self.align_end = True
        else:
            self.msg.linear.x = 0.0 #m/s
            self.msg.angular.z = self.align_max_ang_vel
        
        # Saturate velocity if necessary
        if abs(self.msg.angular.z)>self.align_max_ang_vel: #rad/s
            self.msg.angular.z = + self.align_max_ang_vel * np.sign(self.msg.angular.z)
    # END align_left()
            
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    # action to torn right, move forward and right side
    def align_right(self):
        delta_theta = round(np.rad2deg(self.theta - self.angle_init))

        # The direction in the current robot coordinate frame, toward which the wall is
        self.wall_dir_idx_in_lidar = int((360 + (self.index_init - delta_theta)) % 360)
        
        # Around the "self.wall_dir_idx_in_lidar"
        # [self.wall_dir_idx_in_lidar - self.focus_angle_half, self.wall_dir_idx_in_lidar + self.focus_angle_half]
        limit1 = self.wall_dir_idx_in_lidar - self.focus_angle_half
        limit2 = self.wall_dir_idx_in_lidar + self.focus_angle_half

        minIndex = min(limit1, limit2)
        maxIndex = max(limit1, limit2)

        indeces = list(range(minIndex,maxIndex+1))
        indeces = orderLaserScanSectionIndeces(indeces)

        
        # Run RANSAC algorithm
        pointsForRansac = np.array(self.ranges)[indeces]
        anglesForRansac = np.array(indeces)

        # To test WITH noise
        #inliers_1, outliers_1, self.m, q_best_1, lidar_measurements = ransac(self.threshold, self.iterations, pointsForRansac, anglesForRansac, self.add_noise, self.sigma)
        self.wall_line.find_with_ransac(pointsForRansac,anglesForRansac)

        # Send to topic '/ransac_lidar_view' and to Rviz, the LaserPoints of the lidar section used for RANSAC
        self.pub_ransac_lidar_indeces(indeces, self.wall_line.lidarInputPoints)

        # Use proportional controller only when 'self.wall_dir_idx_in_lidar' is in range [20,160]
        # (this means wall is on robot left). Otherwise use a constant clockwise ang. vel.
        # until wall ('self.wall_dir_idx_in_lidar'), is on the left  (and P control is finally used)
        # (For further explanations see at the bottom of this file, "EXPLANATIONS", point 2) )
        if self.wall_dir_idx_in_lidar in range(20,160+1):
            # Angular coefficient Error = desired m - current m
            err_m = self.m_d - self.wall_line.m

            if abs(err_m)>0.01:
                self.msg.linear.x = 0.0 #m/s
                self.msg.angular.z = - self.K_P * err_m
            
            else:
                self.msg.linear.x = 0.0 #m/s
                self.msg.angular.z = 0.0 #rad/s
                # change state
                self.align_end = True
        else:
            self.msg.linear.x = 0.0 #m/s
            self.msg.angular.z = - self.align_max_ang_vel
        
        # Saturate velocity if necessary
        # If necessary, saturate velocities if they exceed limits
        if abs(self.msg.angular.z)>self.align_max_ang_vel: #rad/s
            self.msg.angular.z = + self.align_max_ang_vel * np.sign(self.msg.angular.z)
    # END align_right()
            
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
    # action to follow the wall, move forward 
    def follow_the_wall(self):
        if self.turn_left:
            indeces = list(range(270-self.focus_angle_half, 270+self.focus_angle_half))
        else:
            indeces = list(range(90-self.focus_angle_half, 90+self.focus_angle_half))
        indeces = [((360+ind)%360) for ind in indeces]
        
        # Run RANSAC algorithm
        pointsForRansac = np.array(self.ranges)[indeces]
        anglesForRansac = np.array(indeces)

        # To test WITH noise
        self.wall_line.find_with_ransac(pointsForRansac, anglesForRansac)

        # Send to topic '/ransac_lidar_view' and to Rviz, the LaserPoints of the lidar section used for RANSAC
        indeces = orderLaserScanSectionIndeces(indeces)
        self.pub_ransac_lidar_indeces(indeces, self.wall_line.lidarInputPoints)

        # Angular coefficient Error = desired m - current m
        err_m = self.m_d - self.wall_line.m

        self.align_max_ang_vel = 0.5 # rad/s

        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.1 #m/s
        #sign = +1 if self.turn_left else -1

        self.msg.angular.z = - self.K_P * err_m
          
        # Saturate velocity if necessary
        # If necessary, saturate velocities if they exceed limits
        if abs(self.msg.angular.z)>self.align_max_ang_vel: #rad/s
            self.msg.angular.z = self.align_max_ang_vel * np.sign(self.msg.angular.z)
    # END follow_the_wall()
            
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def rotate180deg(self):
        # TO DO
        self.msg.linear.x = 0.0 #m/s

        if self.init_theta==None:
            self.init_theta = self.theta
        else:
            delta_theta = abs( self.theta - (self.init_theta+math.pi)%(2*math.pi) )
            # FOR DEBUGGING:
            """print("self.theta: ", self.theta, " self.init_theta: ", self.init_theta)
            print(" (self.init_theta+math.pi)%(2*math.pi): ", (self.init_theta+math.pi)%(2*math.pi), " delta_theta: ", delta_theta, "  <{:.3f}?".format(np.radians(1)))
            """
            if delta_theta < np.radians(1): # The rotation of 180° is completed
                self.msg.angular.z = 0.0 #rad/s
                self.init_theta = None
                self.rot180deg_end = True
                #print("I've turned of 180°!")
            else: # the rotation of 180° is not completed, so go on turning
                self.msg.angular.z = 0.3 #rad/s
    # END rotate180deg()
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def rewind(self):
        # Check for collisions. If so, stop the robot, and interrupt execution
        # (the value is different for simulation an real robot)
        if os.getenv('ROS_DOMAIN_ID')!=None:
            min_dist = 0.22
        else:
            min_dist = 0.06
        if self.regions['narrow_front']<min_dist:
            self.stop_robot()
            raise Exception("\nERROR: execution interrupted since a possible collision has been detected ( min(self.regions['front']) = {:.3f} [m] < {:.3f})".format(self.regions['front'],min_dist) )


        if self.vel_stack: # the stack is not empty
            command = self.vel_stack.pop() # extract in LIFO order, the last velocity (Twist) command from the stack/history
            self.msg.linear.x = command.linear.x
            self.msg.angular.z = -command.angular.z # the opposite ang. vel. to rewind correctly!
        else: # the stack is empty
            raise Exception("ERROR: self.vel_stack is empty") # if the stack is empty, the code above should handle it, without arriving at this error
    # END rewind()
    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def stop_robot(self): # Fabio/Michele adding: to stop the robot when user press CTRL+C SIGTERM
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.publisher_.publish(self.msg)
        self.reset_focus_view()
    # END stop_robot()
    
# END Turtlebot3RobustWallFollower =======================================================================================================================




# START main() ===========================================================================================================================================
def main(args=None):

    # - - - - - - - - -TUNABLE PARAMETERS - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #
    distance_threshold = 0.15   # [m] distance under which the robot detects a wall/obstacle (suggested value for real Turtlebot3 burger: 0.25)
    front_angle_half = 80       # [°] half width of front region, used to detect and find walls in the surroundings (suggested value for real Turtlebot3 burger: 80)
    focus_angle_half = 25       # [°] angle width of the 'narrow_front' and movable region where RANSAC points are extracted  (suggested value for real Turtlebot3 burger: 20)
    side_angle = 20             # [°] width of lateral regions 'left' and 'right'  (suggested value for real Turtlebot3 burger: 20)
    ransac_threshold = 0.01     # distance under which a point gives its consensus to a line (suggested value for real Turtlebot3 burger: 0.001)
    ransac_iterations = 100     # number of iterations of RANSAC algorithm  (suggested value for real Turtlebot3 burger: 200)
    add_noise = False           # choose whether to add white noise to lidar readings  (suggested value for real Turtlebot3 burger: False)
    sigma = 0.01                # standard deviation of the optional added white noise  (suggested value for real Turtlebot3 burger: since 'add_noise' is False, do not matter)
    K_P = 1                     # controller P term  (suggested value for real Turtlebot3 burger: 1)
    align_max_ang_vel = 0.5     # [rad/s] < 1.82! Maximum angular velocity in 'align left/right' state (saturation)  (suggested value for real Turtlebot3 burger: 0.5)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - #

    usage_msg = "\nTo choose if the robot should align to left or right, you must specify it as ros args:\n\n"+\
        "$ ros2 run robust_wall_follower robust_wall_follower --ros-args -p align_direction:=right \n\n"+\
        "or \n\n$ ros2 run robust_wall_follower robust_wall_follower --ros-args -p align_direction:=left"""

    if len(sys.argv)!=4:
        raise Exception("ERROR! Wrong number of input parameters!"+usage_msg)
    
    input("Press <ENTER> to start (ONLY WHEN UNITY IS ALREADY RUNNING)...")

    # The usual code to start, execute and terminate correctly a ROS node...
    rclpy.init(args=args)

    turtlebot3_RobustWallFollower_node = Turtlebot3RobustWallFollower(distance_threshold, front_angle_half, focus_angle_half, side_angle,
                                                    ransac_threshold, ransac_iterations, add_noise, sigma, K_P,align_max_ang_vel)
    
    try:
        rclpy.spin(turtlebot3_RobustWallFollower_node)
    except KeyboardInterrupt:
        turtlebot3_RobustWallFollower_node.stop_robot()
        print("Node terminated by user!")
        time.sleep(0.5)
    finally:
        turtlebot3_RobustWallFollower_node.stop_robot()
        print("Robot stopped!")


    turtlebot3_RobustWallFollower_node.destroy_node()
    rclpy.shutdown()
# END main() =============================================================================================================================================


if __name__ == '__main__':
    main()



"""
                              EXPLANATIONS:

1)  Boolean variable 'self.rot180_to_rewind' is a sort of "traffic light",
    that is necessary to specify if the transition EXITING FROM 'rotate 180 deg'
    GOES TO 'rewind' OR INSTEAD 'find wall' state.
    In fact these two edge would have the same condition:
    exit from 'rotate 180 deg' state IF 'self.rot180deg_end'==True
        i.e. (the rotation of 180° has terminated)
    
    However, this condition is not enough to specify if the direction of
    exit is toward 'rewind' or 'find wall'. So, this boolean variable
    'self.rot180_to_rewind', exactly discriminate between these two directions.
    
    SUMMING UP: this variable is needed to discriminate between the direction
    of exit from state 'rotate 180 deg':
     - self.rot180_to_rewind = True  :  'rotate 180 deg' --->  TOWARD 'rewind'
     - self.rot180_to_rewind = False :  'rotate 180 deg' --->  TOWARD 'find wall'

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

2)  In states 'align left/right', an angular velocity is provided to the robot, such that
    it alignes to the wall in the right direction (in 'align left' robot keeps wall on its right,
    while in 'align right' Turtlebot3 keeps wall on its left).
    To command the exact orientation of the Turtlebot3, with respect to the wall, we use a method that
    consists in using a proportional (P) control, where input is the line angular coefficient error
    err_m = m_des - m_curr, desired angular coefficient - current angular coefficient,
    and the controller output is interpreted as an angular velocity command. 

    However such a controller cannot be used naively, for 2 main reasons:    
        - line do not have orientation (so the robot can align to a wall with two orientations
          aligned to the wall, or rotated of 180° and still be aligned to the wall);
        - line equation is problematic: the angular coefficient value presents discontinuities near to
            parallel lines to the y axis, where the angular coefficient change abruptly from -∞ to +∞;
    
    These two facts, raise problems which make impossible to use this simple proportional controller.

    For instance, in 'align left' robot should keep the wall on its right. Lets suppose instead that is perfectly
    aligned with the wall on its left. Using a "naive" P controller, would provide a null velocity, since
    the error between the desired angular velocity, minus the current one is 0.
    And that's not the desired behaviour, since we would like the robot rotates of 180°.

    Or another problem: lets start again from the same position described above, with wall on the left, which is
    in the opposite direction of the desired one. (Consider that robot pointing direction, coincides with the x axis!)
    Lets suppose the velocity input, based on ang. coeff. error is right: robot starts rotating, at a certain point
    robot is almost aligned PERPENDICULARLY to the wall, so the current ang. coeff. m, has a value tending to ±∞.
    Error has ∞ magnitude, that at least must be saturated. But this is not the only problem. When robot keeps rotating,
    sooner or later it overcomes the perpendicular direction to the wall, and so line ang. coeff. will have an opposite
    sign (∓∞)! This means the error will has a different sign and an angular velocity in the opposite direction!
    The result, is that robot remains stuck in this position, oscillating back and forth, because before a positive ang.
    velocity is commanded, and then a positive one.

    Thus, the SOLUTION consists in limiting the range of angles in which the proportional controller is used:
    in this region the angular coefficient has continuous and finite values, which can be used to compute a
    continuous and limited error, which is the input of the proportional controller.
    On the other hand, in other regions, a constant angular velocity is provided, regardless of current line coefficient.
    This region is respectively [20°,160°] for the 'align right' state. In fact, when wall is approximately on the
    robot left <-> [20°,160°], then the porportional controller is used, ready also for recovering from possible overshoots.
    Similarly, for 'align left' state, wall must be in direction [200°,340°], that is, on the right.
    (Notice: when talking about robot direction, it is intended the closest distance measured by the lidar, whose index
     indicates the relative wall position, with respect to robot reference frame!).

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

3)  the biggest problem in migrating from simulation to real Turtlebot3, was due to the lidar.
    In simulation the lidar always provides 361 array elements. In this way, the points recorded
    are expressed like in a polar coordinate system. The i-th element of the lidar, is the
    i-th point whose content is "ρ", the Euclidean distance of the point from the origin, while
    the index i, express the angle "θ" with respect to the x axis (counter-clockwise positive).
    Since the length of the lidar array return is always constant, these assumptions easily hold.
    However, in reality, that's not the case. The lidar readings array returned, has a variable
    length! For instance during experiments, the length varied from 220 to 240, for each
    lidar iteration.
    This complicates extremely things, because, losing the index, means losing the angular
    coordinate "θ" of the points, which are completely wrong.
    For instance, let's suppose the lidar loses a reading, each three: at the end, the array
    will be 240. This means that the reading relative to angle at index 3, that in Turtlebot3
    correspond to 3°, is lost, and the reading of the angle 4°, index 4, takes its place!
    Going on, until the end of the lidar, the reading taken at angle 360°, will be in the array
    position index 240! It is clear so, that angular component cannot be considered reliable
    anymore!
    Consequently, our SOLUTION, is implemented in "functions/auxiliary_functions.py", function
    "add_lidar_readings".
    The main idea is that, we assume that lidar readings are lost in average at regular intervals.
    So, at regular equispaced intervals, we take two consecutive lidar readings, we computed the
    mean between them, and we put the result in an additional element, which is inserted into
    the array, in between them. In this way, every time a new element is added, lidar readings array
    increases its dimension of 1, and the procedure is repeated a precise number of time, with
    a precise interval, such that the final length of the array is exactly 360. In this way, all
    the plane is covered, with 360° width, and points are approximately in the position where
    actually they are in reality.
    Obviously this method works only under the assumption that lidar readings are lost at regular
    time/angle interval.
    But the experimental tests, have proved that this hypothesis is right, and the behaviour
    of the algorithm, thanks to this solution, is exactly the desired one!

  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

"""