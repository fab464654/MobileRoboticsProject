# LAB 4: 25th May 2022
# Exercise 1.c: REWIND

from cmath import nan
import rclpy
from rclpy.node import Node
import rclpy.qos
from pynput import keyboard

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import numpy as np
import math

import time
import threading
# My addings:
import sys
from collections import deque
import copy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# Fabio/Michele adding
# Function for the conversion from quaternion to euler angles (orient.)
def quat2eul(w, x, y, z, unit="rad"):
    assert unit=="rad" or unit=="deg", """ERROR: quat2eul() param 'unit' must be "rad" or "deg"! """
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    if unit=="deg":
        X = np.degrees(np.arctan2(t0, t1))
    else:
        X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    if unit=="deg":
        Y = np.degrees(np.arcsin(t2))
    else:
        Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    if unit=="deg":
        Z = np.degrees(np.arctan2(t3, t4))
    else:
        Z = np.arctan2(t3, t4)
    
    #print("(",unit,") X: ", X, ", Y: ", Y , ", Z: ", Z)

    return X, Y, Z



class Turtlebot3HighLevelControl(Node):

    def __init__(self, turn_left):
        super().__init__('turtlebot3_HighLevelControl_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, rclpy.qos.qos_profile_sensor_data)
        # Publisher on topic '/front', '/left' and '/right' the subset of lidar points from the relative
        # regions, to which the noise could have been added
        self.front_pub = self.create_publisher(LaserScan, '/front', 1)
        self.left_pub = self.create_publisher(LaserScan, '/left', 1)
        self.right_pub = self.create_publisher(LaserScan, '/right', 1)
        # Publisher on topic '/current_state', where a simple String representing the current state
        # of the FSM (Finite State Machine) is published, in order to show it in RViz2
        self.current_state_str_pub = self.create_publisher(Marker, '/current_state', 1)
        # Publisher for representing in RViz2 the path followed by the Turtlebot3
        self.path_pub = self.create_publisher(Path, '/path', 1)
        
        # Initialize keyboard Listener to detect keys inserted by user
        keyboard.Listener(on_press=self.press_callback).start()
        self.key_pressed = False # this boolean variable is set to True when user insert any key

        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
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

        # distance threshold to the wall
        self.th = 0.15 # [m] (at least 0.25 in real turtlebot3 burger)

        timer_period = 0.1  # [seconds]

        # Initializations of:
        self.twist = Twist() # current robot twist (linear+angular velocity), from /odom topic
        self.theta = 0.0 # current orientation of the robot, extracted from '/odom' topic
        self.init_theta = None # initial theta. Used to make the robot rotating exactly of 180°: (current orientation - self.init_theta) must be 180°
        self.rot180deg_end = False # boolean var. to specify when the rotation of 180° of the robot has ended
        self.rot180_to_rewind = True # description below (represents the state before "rotate 180°" or equivalently, the state before pressing a button)
        # This boolean variable 'self.rot180_to_rewind' is a sort of "traffic light",
        # that is necessary to specify if the transition EXITING FROM 'rotate 180 deg'
        # GOES TO 'rewind' OR INSTEAD 'find wall' state.
        # In fact these two edge would have the same condition:
        # exit from 'rotate 180 deg' state IF 'self.rot180deg_end'==True
        #       i.e. (the rotation of 180° has terminated)
        #
        # However, this condition is not enough to specify if the direction of
        # exit is toward 'rewind' or 'find wall'. So, this boolean variable
        # 'self.rot180_to_rewind', exactly discriminate between these two directions.
        #
        # SUMMING UP: this variable is needed to discriminate between the direction
        # of exit from state 'rotate 180 deg':
        # - self.rot180_to_rewind = True  :  'rotate 180 deg' --->  TOWARD 'rewind'
        # - self.rot180_to_rewind = False :  'rotate 180 deg' --->  TOWARD 'find wall'

        # Data structure to implement a STACK, in which the velocitis of each "clock time"
        # are saved/stored. These will be used in 'rewind' state to repeat past actions
        self.vel_stack = deque(maxlen=1800) # stack saves velocities for at maximum 3 minutes (see below) (just a choice)
        # timer_period = 0.1 sec -> 10 cycles/sec -> 600 cycles/min
        # So, for instance, in 3 min -> 1800 cycles

        self.turn_left = turn_left # when False robot keeps wall on its right (set by command line argument)

        self.path = Path()

        self.timer = self.create_timer(timer_period, self.control_loop)
    

    # Method for listening to the keyboard
    def press_callback(self,key): # PAY ATTENTION TO THE PARAMETERS ORDER!!!
        if key==keyboard.KeyCode.from_char('r'):
            if self.state_!=4:
                self.key_pressed = True # always read from keyboard, except for when
                                        # the robot is rotating of 180° (self.state_==4,
                                        # 'rotate 180 deg'), This is an "interruptible"
                                        # routine (at least I designed it like that)
        else:
            print("Only allowed char is 'r', to start/stop <rewind> procedure!\n")
    
    
    # To read from robot odometry the actual current robot orientation (self.theta)
    def odom_callback(self,msg):
        # Quaternion expressing the orientation
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        rpy = quat2eul(x,y,z,w) # conversion quaternion -> Euler angles (2nd angle is the only interesting one for us)

        # Just to check if velocity command has been correctly performed
        # self.twist = msg.twist.twist

        self.theta = rpy[0]%(2*math.pi) # theta in range [0,2*pi)
        
        # Append current Pose to self.path variable, to visualize the path in RViz2
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)
    

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

        msg.scale.x = 0.13
        msg.scale.y = 0.13
        msg.scale.z = 0.13

        msg.color.r = 0.0#0.3
        msg.color.g = 0.0#0.4
        msg.color.b = 0.5#1.0
        msg.color.a = 1.0
        self.current_state_str_pub.publish(msg)
    # END publish_current_state_marker()


    # loop each 0.1 seconds
    def control_loop(self):

        # Check whether the previous velocity command send to the robot has been executed correctly. If yes, save it
        """save_vel = abs(self.msg.linear.x - self.lin_vel) < 0.01
        print("abs(self.msg.linear.x - self.lin_vel) ", abs(self.msg.linear.x - self.lin_vel), "< 0.01")
        save_vel = save_vel and abs(self.msg.angular.z - self.ang_vel) < 0.01
        print("abs(self.msg.angular.z - self.ang_vel) ", abs(self.msg.angular.z - self.ang_vel), "< 0.01", end="\n\n")"""
    
        # actions for states 
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
        
        self.publisher_.publish(self.msg)

        # To publish a string representing the current state to topic '/current_state' subscripted by Rviz
        self.publish_current_state_marker()

            
    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 
        
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TUNE INSIDE HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # FUNDAMENTAL PARAMETER!!! (Fabio/Michele)
        # Half angle width of the 'front' region
        #                                            (    d     )
        #        front_angle_half = arccos_INDEGREES(-----------) - 90°
        #                                           ( self.th )
        # d: (min) distance robot-wall
        front_angle_half = 55 # PAY ATTENTION IN CHANGING IT (SUGGESTED: in simulation 55, in real Trutlebot3 burger 32)

        side_angle = 80 # angle width of regions 'left' and 'right' in degrees (in simul. 80)
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        # Initialization of the three LaserScan() messages relative to the lidar
        # readings (point cloud) relative respectively to region 'front', 'left' and 'right'
        length = len(msg.ranges)
        front_msg = copy.deepcopy(msg)
        left_msg = copy.deepcopy(msg)
        right_msg = copy.deepcopy(msg)

        # if index i is in front/left/right region, then
        #     keep this readings + add white noise
        # otherwise,
        #     set this reading to 0.0
        #
        # In this way, for each region, are only kept the (noisy) readings relative to
        # that regions, while the others are "deleted"

        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TUNE INSIDE HERE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        add_noise = False       # ADD NOISE ?
        sigma = 0.01            # standard deviation of the normal error (mean 0) eventually added
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        for i in range(length):
            # Front region
            if not i in range(front_angle_half) and not i in range(length-front_angle_half, length):
                front_msg.ranges[i] = np.nan
            elif add_noise:
                front_msg.ranges[i] = front_msg.ranges[i] + np.random.normal(0,sigma)
            # else: keep the value unchanged

            # Left region
            if not i in range(front_angle_half, front_angle_half+side_angle):
                left_msg.ranges[i] = np.nan
            elif add_noise:
                left_msg.ranges[i] = left_msg.ranges[i] + np.random.normal(0,sigma)
            # else: keep the value unchanged

            # Right region
            if not i in range(length-front_angle_half-1-side_angle, length-front_angle_half-1):
                right_msg.ranges[i] = np.nan
            elif add_noise:
                right_msg.ranges[i] = right_msg.ranges[i] + np.random.normal(0,sigma)
            # else: keep the value unchanged


        # Publish the LaserScan() messages relative to 'front', 'left' and 'right'
        self.front_pub.publish(front_msg)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        # From subregions  'front', 'left' and 'right', extract the minimum distance
        # measured there
        self.regions = {
            'front':  np.nanmin(front_msg.ranges),#min(min(min(ranges[:front_angle_half]),10), min(min(ranges[-front_angle_half:]), 10)),
            'left':   np.nanmin(left_msg.ranges),#min(min(ranges[front_angle_half:front_angle_half+side_angle]), 10),
            'right':  np.nanmin(right_msg.ranges),#min(min(ranges[len(ranges)-front_angle_half-1-side_angle:len(ranges)-front_angle_half-1]), 10),
            # it is like there is a "back" region, but it is useless to define it, since we would not use it anyway
        }

        # function where are definied the rules for the change state
        self.take_action()


    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call change_state function with the state index to enable the change state

        # 0: 'find the wall', 1: 'align left', 2: 'follow wall'
        #print("self.key_pressed: ", self.key_pressed, " self.state_: ", self.state_)

        # EDGE THAT ENDS IN 'rotate 180 deg' STATE (Fabio/Michele: additional state and transition)
        if self.state_ in [0,1,2,3] and self.key_pressed:
            self.key_pressed = False
            """if len(self.vel_stack)>=3: # FOR DEBUGGING
                print("self.vel_stack: [0]: lin. vel. ", round(self.vel_stack[0].linear.x,2), ", ang. vel. ", round(self.vel_stack[0].angular.z,2), "\n",
                      "                [1]: lin. vel. ", round(self.vel_stack[1].linear.x,2), ", ang. vel. ", round(self.vel_stack[1].angular.z,2), "\n",
                      "                [2]: lin. vel. ", round(self.vel_stack[2].linear.x,2), ", ang. vel. ", round(self.vel_stack[2].angular.z,2), "\n",
                      "                [3]: lin. vel. ", round(self.vel_stack[3].linear.x,2), ", ang. vel. ", round(self.vel_stack[3].angular.z,2), end="\n\n")
            """
            self.change_state(4)
        
        # EDGE 'rotate 180 deg' ---> 'rewind'
        elif self.state_==4 and self.rot180deg_end and self.rot180_to_rewind:
            self.rot180deg_end = False
            self.rot180_to_rewind = False
            self.change_state(5) # E4
        
        # EDGE 'rotate 180 deg' ---> 'find wall'
        elif self.state_==4 and self.rot180deg_end and not self.rot180_to_rewind:
            self.rot180deg_end = False
            self.rot180_to_rewind = True
            self.change_state(0) # E5
        
        # EDGE 'rewind' ---> 'rotate 180 deg'
        elif self.state_==5 and (self.key_pressed or len(self.vel_stack)==0): # exit from 'rewind' if user has pressed a key or velocities history is empty
            self.key_pressed = False
            if len(self.vel_stack)==0:
                print("Velocities history empty! Resuming 'find the wall' task...")
            self.change_state(4) # rotate of 180° before to restart the normal execution
        
        # EDGE THAT GOES TO STATE 'find_wall' FROM STATE 'align_left' OR 'follow wall'
        elif (self.state_==1 or self.state_==2) and self.regions['right']>self.th and self.regions['front']>self.th and self.turn_left: # I ADDED the SECOND-LAST CONDITION: search the wall only if it is not already in front
            self.change_state(0) # E1

        # EDGE THAT GOES TO STATE 'find_wall' FROM STATE 'align_right' OR 'follow wall'
        elif (self.state_==2 or self.state_==3) and self.regions['left']>self.th and self.regions['front']>self.th and not self.turn_left:
            # if the wall on robot left that it has been following up to now, has just ended
            # and so another wall to follow must be searched
            self.change_state(0)
        
        # EDGES WHICH GO TO STATE 'align_left'
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']>self.th and self.turn_left:
            self.change_state(1) # E2 - a
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']<self.th and self.turn_left:
            self.change_state(1) # E2 - b
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']<self.th and self.turn_left:
            self.change_state(1) # E2 - c
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']>self.th and self.turn_left: # MY ADD: read below
            self.change_state(1) # E2 - d (MY ADD: even if in the right there is free space, but in front/left there is a wall, stop it and align to it!)
        
        # EDGES WHICH GO TO STATE 'align_right' ("specular" to conditions above of 'align_left')
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']>self.th and not self.turn_left:
            self.change_state(3)
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']>self.th and not self.turn_left:
            self.change_state(3)
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']<self.th and not self.turn_left:
            self.change_state(3)
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']<self.th:
            self.change_state(3) # even if to the left there is free space, but in front/right there is a wall, stop it and align to it!)
        
        # EDGE THAT GOES TO STATE 'follow_wall' FROM 'find_wall' OR 'align_left'
        elif (self.state_==0 or self.state_==1) and self.regions['front']>self.th and self.regions['right']<self.th and self.turn_left:# and self.regions['right']<self.th:
            self.change_state(2) # E3

        # EDGE THAT GOES TO STATE 'follow_wall' FROM 'find_wall' OR 'align_right'
        elif (self.state_==0 or self.state_==3) and self.regions['front']>self.th and self.regions['left']<self.th and not self.turn_left:# and self.regions['right']>self.th:
            self.change_state(2)

        else:
            pass # FSM remains in the same state
            #print(self.regions)


        # FOR DEBUGGING:
        print("State: <", self.state_dict_[self.state_], ">, front: ", round(self.regions['front'],3), " [m], left: ", round(self.regions['left'],3), " [m], right: ", round(self.regions['right'],3), " [m]\n",\
              "          self.theta: ", np.around(np.degrees(self.theta)) ,"°, lin.v: ", round(self.msg.linear.x,3), " [m/s], ang. v: ", round(self.msg.angular.z,3), " [rad/s]\n",\
              "          len(self.vel_stack): ", len(self.vel_stack), end="\n\n" )
        


    # function to update state
    # don't modify the function
    def change_state(self, state):
        
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state


    # action to find the wall, move forward and wall side to find the wall
    def find_wall(self):

        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.08 #m/s

        if self.turn_left:
            self.msg.angular.z = -0.5 #rad/s
        else:
            self.msg.angular.z = 0.5 #rad/s
        

    # action to torn left, move forward and left side
    def align_left(self):
        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.0 #m/s
        self.msg.angular.z = 0.23 #rad/s # I THINK IT IS BETTER TO KEEP IT LOW

    
    # action to torn right, move forward and right side
    def align_right(self):
        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.0 #m/s
        self.msg.angular.z = -0.23 #rad/s # I THINK IT IS BETTER TO KEEP IT LOW

    
    # action to follow the wall, move forward 
    def follow_the_wall(self):
        
        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.1 #m/s
        self.msg.angular.z = 0.0 #rad/s

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
    
    def rewind(self):
        # Check for collisions. If so, stop the robot, and interrupt execution
        min_dist = 0.05
        if self.regions['front']<min_dist:
            self.stop_robot()
            raise Exception("\nERROR: execution interrupted since a possible collision has been detected ( min(self.regions['front']) = {:.3f} [m] < {:.3f})".format(self.regions['front'],min_dist) )


        if self.vel_stack: # the stack is not empty
            command = self.vel_stack.pop() # extract in LIFO order, the last velocity (Twist) command from the stack/history
            self.msg.linear.x = command.linear.x
            self.msg.angular.z = -command.angular.z # the opposite ang. vel. to rewind correctly!
        else: # the stack is empty
            raise Exception("ERROR: self.vel_stack is empty") # if the stack is empty, the code above should handle it, without arriving at this error
        
    def stop_robot(self): # Fabio/Michele adding: to stop the robot when user press CTRL+C SIGTERM
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.publisher_.publish(self.msg)


def main(args=None):

    # To choose if the robot should align to left or right there are two options:
    # 1) modify manually the variable 'turn_left' below to choose the alignment direction.
    #    REMEMBER: after every change, you should compile the ros workspace to apply changes.
    #    To execute this case, you should not pass any parameter in the command window!
    # 2) insert as command arguments either "-- left" or "-- right" to specify at run time
    #    the desired alignment direction (no compilation is needed!)

    # ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    # !               MODIFY HERE BOOLEAN VARIABLE 'turn_left'                    !
    # !  to choose in which direction the robot must turn when it faces a wall    !
    # !   - 'turn_left' == True: the robot turn left when faces a wall            !
    # !                          and so it keeps it on its right side             !
    # !   - 'turn_left' == False: the robot turn right, keeping the               !
    # !                           wall on its left side;                          !
    # ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    turn_left = False
    
    # Check number of parameters, must not exceeds 2
    assert len(sys.argv)<=2, '(Michele error) no more than 1 argument can be passed (you can insert: nothing or "-- left" or "-- right"!) '
    # Rememeber: also $'ros2 run node node' is considered an argument. So if user insert another one, they are 2

    # Case 2) overwrite align direction according to the command line argument chosen by the user
    if len(sys.argv)==2:
        assert sys.argv[1]=="left" or sys.argv[1]=="right", '(Michele error) argument must be exactly either "-- left" or "-- right"! '
        turn_left = sys.argv[1]=="left"
        my_str = "|            USER HAS SPECIFIED THAT ROBOT MUST ALIGN TO " + (" LEFT" if sys.argv[1]=="left" else "RIGHT") + "!               |"
        my_str_width = len(my_str)
        border = "+" + ("-"*(my_str_width-2)) + "+"
    else: # Case 1) use variable 'turn_left' value to specify manually the alignment direction (no argument given)
        my_str = "|     REMEMBER: you can CHOOSE robot ALIGNMENT with command line argument:    |"
        my_str_width = len(my_str)
        my_str = my_str + "\n" + \
                 "| $ ros2 run turtlebot3_HighLevelControl turtlebot3_HighLevelControl -- left  |"
        my_str = my_str + "\n" + \
                 "| $ ros2 run turtlebot3_HighLevelControl turtlebot3_HighLevelControl -- right |"
        border = "+" + ("-"*(my_str_width-2)) + "+"
    
    # Just a message to print in the terminal (to work correctly, the code must be executed
    # only when Unity is already running, otherwise the stack collects wrong Twist velocities)
    reminder = "|" + (" "*(my_str_width-2)) + "|\n"+\
               "|    You can press anytime you want the key 'r 'on the keyboard, to           |\n"+\
               "|    start the REWIND procedure, in which robot redo its previous             |\n"+\
               "|    movements. Pressing  again 'r' during 'rewind' state, interrupts         |\n"+\
               "|    this routine, starting again with the 'follow wall' task.                |\n"+\
               "|" + (" "*(my_str_width-2)) + "|\n"+\
               "|     PAY ATTENTION: CLICK <play> button in Unity AND WAIT for ITS START      |\n"+\
               "|                    BEFORE to press <ENTER> for executing this node!         |"
    print(border,my_str,reminder,border,sep="\n",end="\n\n")
    input("Press <ENTER> to start (WHEN UNITY IS ALREADY RUNNING)...")

    # The usual code to start, execute and terminate correctly a ROS node...
    rclpy.init(args=args)

    turtlebot3_HighLevelControl_node = Turtlebot3HighLevelControl(turn_left)
    
    try:
        rclpy.spin(turtlebot3_HighLevelControl_node)
    except KeyboardInterrupt:
        turtlebot3_HighLevelControl_node.stop_robot()
        print("Node terminated by user!")
        time.sleep(0.5)


    turtlebot3_HighLevelControl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
