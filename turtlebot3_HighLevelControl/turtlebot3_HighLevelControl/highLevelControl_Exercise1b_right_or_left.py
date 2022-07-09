# LAB 4: 25th May 2022
# Exercise 1.b: FOLLOW THE WALL KEEPING IT EITHER ON THE RIGHT SIDE
#               OF THE ROBOT, OR ON THE LEFT SIDE.
#               To choose on which side, set the value of the boolean
#               variable 'turn_left':
#               - 'turn_left' == True: the robot turn left when faces a wall
#                                       and so it keeps it on its right
#               - 'turn_left' == False: the robot turn right, keeping the
#                                       wall on its left

import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

import time
import threading
import sys # my add, to insert command line argument

class Turtlebot3HighLevelControl(Node):

    def __init__(self, turn_left=True):
        super().__init__('turtlebot3_HighLevelControl_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        
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
        }
        # velocity command
        self.msg = Twist()

        # distance threshold to the wall
        self.th = 0.15 # [m]

        timer_period = 0.1  # [seconds]

        # Variable that specifies the alignment direction
        self.turn_left = turn_left # MY ADD

        self.timer = self.create_timer(timer_period, self.control_loop)


    # loop each 0.1 seconds
    def control_loop(self):
    
        # actions for states 
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
        elif self.state_ == 3:
            self.align_right()
        else:
            print('Unknown state!')
        
        self.publisher_.publish(self.msg)

            
    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 
        ranges = msg.ranges.tolist()
        
        # FUNDAMENTAL PARAMETER!!! (Fabio/Michele)
        # Half angle width of the 'front' region
        #                                            (    d     )
        #        front_angle_half = arccos_INDEGREES(-----------) - 90°
        #                                           ( self.th )
        # d: (min) distance robot-wall
        front_angle_half = 55 # DO NOT CHANGE IT (in simulation)

        side_angle = 80 # angle width of regions 'left' and 'right' in degrees

        
        self.regions = {
        'front':  min(min(min(ranges[:front_angle_half]),10), min(min(ranges[-front_angle_half:]), 10)),
        'left':  min(min(ranges[front_angle_half:front_angle_half+side_angle]), 10),
        'right':  min(min(ranges[len(ranges)-front_angle_half-1-side_angle:len(ranges)-front_angle_half-1]), 10),
        # 'back': min(min(ranges[135:225]),10) # is like there is this region, but it is useless to define it
        }

        # function where are definied the rules for the change state
        self.take_action()


    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call change_state function with the state index to enable the change state

        # 0: 'find the wall', 1: 'align left', 2: 'follow wall', 3: 'align right'

        # EDGE THAT GOES TO STATE 'find the wall' FROM STATE 'align_left' OR 'follow wall'
        if (self.state_==1 or self.state_==2) and self.regions['right']>self.th and self.regions['front']>self.th and self.turn_left: # I ADDED the LAST CONDITION: search the wall only if it is not already in front
            self.change_state(0) # E1

        # EDGE THAT GOES TO STATE 'find the wall' FROM STATE 'align_right' OR 'follow wall'
        if (self.state_==2 or self.state_==3) and self.regions['left']>self.th and self.regions['front']>self.th and not self.turn_left:
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
        print("State:", self.state_dict_[self.state_], " front: ", round(self.regions['front'],3), " left: ", round(self.regions['left'],3), " right: ", round(self.regions['right'],3), "lin.v: ", round(self.msg.linear.x,3), "ang. v: ", round(self.msg.angular.z,3) )



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

    # Method to stop the robot (MY ADD)
    def stop_robot(self):
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
    # !      FOR MANUAL SETTING OF THE ALIGNMENT DIRECTION (POINT 1) above)       !
    # !  to choose in which direction the robot must turn when it faces a wall    !
    # !   - 'turn_left' == True: the robot turn left when faces a wall            !
    # !                          and so it keeps it on its right side             !
    # !   - 'turn_left' == False: the robot turn right, keeping the               !
    # !                           wall on its left side;                          !
    # ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    turn_left = False
    
    # Check number of command line arguments (must be <=2)
    # Rememeber: also $'ros2 run package_name node_name' is considered the first argument
    # So, everything inserted after -- is considered as custom user argument, and so the number of parameters is 2 in this last case (not 1)
    assert len(sys.argv)<=2, '(Michele error) no more than 1 argument can be passed (you can insert: nothing or "-- left" or "-- right"!) '
    if len(sys.argv)==2: # Case 2) in which alignment direction is choosen from command line argument
        assert sys.argv[1]=="left" or sys.argv[1]=="right", '(Michele error) argument must be exactly either "-- left" or "-- right"! '
        turn_left = sys.argv[1]=="left"
        my_str = "|            USER HAS SPECIFIED THAT ROBOT MUST ALIGN TO " + (" LEFT" if sys.argv[1]=="left" else "RIGHT") + "!               |"
        my_str_width = len(my_str)
        border = "+" + ("-"*(my_str_width-2)) + "+"
    else: # Case 1) in which alignment direction has been choosen manually according to 'turn_left' variable above
        my_str = "|     REMEMBER: you can CHOOSE robot ALIGNMENT with command line argument:    |"
        my_str_width = len(my_str)
        my_str = my_str + "\n" + \
                 "| $ ros2 run turtlebot3_HighLevelControl turtlebot3_HighLevelControl -- left  |"
        my_str = my_str + "\n" + \
                 "| $ ros2 run turtlebot3_HighLevelControl turtlebot3_HighLevelControl -- right |"
        border = "+" + ("-"*(my_str_width-2)) + "+"
    print(border,my_str,border,sep="\n",end="\n\n")

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
