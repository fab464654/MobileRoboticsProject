# LAB 4: 25th May 2022
# Exercise 1.a: FOLLOW THE WALL, KEEPING IT ON THE RIGHT SIDE OF THE ROBOT

import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

import time
import threading

class Turtlebot3HighLevelControl(Node):

    def __init__(self):
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
        }
        # velocity command
        self.msg = Twist()

        # distance threshold to the wall
        self.th = 0.15

        timer_period = 0.1  # seconds

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
            pass
        else:
            print('Unknown state!')
        
        self.publisher_.publish(self.msg)

            
    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 
        ranges = msg.ranges.tolist()

        # FUNDAMENTAL PARAMETER!!! (Michele/Fabio)
        # HOW TO FIND THE RIGHT VALUE? Given the threshold, self.th, check at which distance the robot
        #        stops, with respect to the wall, let's call it "d".
        #                                            (    d     )
        #        front_angle_half = arccos_INDEGREES(-----------) - 90°
        #                                           ( self.th )
        #
        # IN SIMULATION KEEP THE VALUE 50(°), otherwise the robot will not be able anymore
        # to stay aligned to the wall during the 'follow wall' state!
        # 
        # This algorithm is really sensible to this parameter, but also to the angular
        # velocity in the `align left` state, in which the robot is rotating.
        # Indeed, if the velocity is too high, the orientation exceeds the target
        # position, with a sort of overshoot!
        front_angle_half = 50 

        side_angle = 85 # angle width of regions 'left' and 'right' in degrees
        # E.g. left region goes from [front_angle_half; front_angle_half+side_angle]

        
        self.regions = {
        'front':  min(min(min(ranges[:front_angle_half]),10), min(min(ranges[-front_angle_half:]), 10)),
        'left':  min(min(ranges[front_angle_half:front_angle_half+side_angle]), 10),
        'right':  min(min(ranges[225:len(ranges)-front_angle_half-1]), 10),
        # 'back': min(min(ranges[135:225]),10) # MY ADD: read below

        # PROBLEM STATEMENT AND SOLUTION (with above line): (Michele/Fabio)
        # Without the introduction of a 4th region pointing backward, i.e. 'back' there
        # could be some problems.
        # In particular, when the robot is in a corner,
        # there could be problems, since left region, comprises also the
        # back of the robot (60° to 180°!), and so, it could seems that there is an
        # obstacle/wall on the left, even if there is no actually on the left but
        # only "in backwards"! This is the case when, after having found a corner, the 'align left'
        # state makes the robot rotating, but not stopping since in its back there is a
        # too close wall, the one that has been just followed. This implies that it
        # seems there is an obstacle on the left, and consequently is not possible to
        # move from 'align left' state to 'follow wall' state! And this, sooner
        # or later will generate some issue, or at least makes the robot move in the
        # wrong direction, not following the right wall.
        }

        # function where are definied the rules for the change state
        self.take_action()


    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call change_state function with the state index to enable the change state

        # 0: 'find the wall', 1: 'align left', 2: 'follow wall'

        # EDGE: 'align left' OR 'follow wall' ---> 'find wall'
        if (self.state_==1 or self.state_==2) and self.regions['right']>self.th and self.regions['front']>self.th: # I ADDED the LAST CONDITION: search the wall only if it is not already in front
            self.change_state(0) # E1
        
        # EDGES: 'find wall' OR 'follow wall' ---> 'align left'
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']>self.th:
            self.change_state(1) # E2 - a
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']>self.th and self.regions['right']<self.th:
            self.change_state(1) # E2 - b
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']<self.th:
            self.change_state(1) # E2 - c
        elif (self.state_==0 or self.state_==2) and self.regions['front']<self.th and self.regions['left']<self.th and self.regions['right']>self.th: # MY ADD: read below
            self.change_state(1) # E2 - d (MY ADD: even if in right there is free space, but in front/left there is a wall, stop it and align to it!)
        #elif self.state_==2 and self.regions['right']<self.th:
        #    self.change_state(1) # MY ADD: if the robot is following the wall, but it is getting too close to the wall itself, go to state 'align left' for going further
        
        # EDGE:  'find wall' OR 'align left' ---> 'follow wall'
        elif (self.state_==0 or self.state_==1) and self.regions['front']>self.th and self.regions['left']>self.th and self.regions['right']<self.th:
            self.change_state(2) # E3      
        #else:
            # remaining in the same state
            #print(self.regions)

        # FOR DEBUGGING:
        print("State:", self.state_dict_[self.state_], " front: ", round(self.regions['front'],3), " left: ", round(self.regions['left'],3), " right: ", round(self.regions['right'],3) )



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
        self.msg.angular.z = -0.5 #rad/s

    # action to torn left, move forward and left side
    def align_left(self):
        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.0 #m/s
        self.msg.angular.z = 0.23 #rad/s # I THINK IT IS BETTER TO KEEP IT LOW

    
    # action to follow the wall, move forward 
    def follow_the_wall(self):
        
        # write velocity commands using the class variable self.msg
        self.msg.linear.x = 0.1 #m/s
        self.msg.angular.z = 0.0 #rad/s

        # PROBLEM: following the wall, the robot usually get further from it,
        # especially choosing a different value for 'front_angle_half' and
        # increasing the magnitude of the angular velocity in state 'align left'.
        # Anyway, with these values, it seems also an insignificant problem, with no
        # dangerous drawbacks.

        # POSSIBLE SOLUTION:
        # I added a NEGATIVE ANGULAR VELOCITY to the 'find wall' state. such that,
        # if the robot get further from the wall, during the 'follow wall' sate, than,
        # passing to the 'find wall' state, this brings back the robot toward the wall


    # Method to stop the robot (MY ADD)
    def stop_robot(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.publisher_.publish(self.msg)



def main(args=None):
    rclpy.init(args=args)

    turtlebot3_HighLevelControl_node = Turtlebot3HighLevelControl()

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
