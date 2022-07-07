# MobileRoboticsProject

## _How to use it:_
To run the implemented code:
- Run Unity and click on the play button (using the "Turtlebot3UnityROS2" map)
- Run the launch file to start Rviz2:
```sh
$ ros2 launch robust_wall_follower_real robust_wall_follower.launch.py
```
- Run the implemented ROS2 node (choosing the desired align_direction left/right):
```sh
ros2 run robust_wall_follower_real robust_wall_follower_real --ros-args -p align_direction:=left
```

## _About the problem:_
The aim of this project was implementing a robust method for moving parallel to a certain wall. In particular, it was an extention of the "High level control - 1.c" already implemented during laboratory session. What can be clearly seen testing the standard wall follower (both in simulation and on the real robot) is that during the "Follow wall" state, the robot continuously oscillates trying to move straight. This behaviour was obtained imposing an angular velocity that causes the robot to turn towards the wall, avoiding to go away from it. The non-perfect alignment is due to odometry/LIDAR readings that are affected by noise and errors. 

In the following image, the implemented Finite State Machine is shown. The algorithm has the ability to perform alignment towards left or right. Also, when a key is pressed during the execution, the robot will turn by 180° ("Rotate 180°" state) and enter the "Rewind" state. This state makes the robot "going back" re-running a certain number of linear/angular velocities according to the values stored in a deque stack.

## _Our solution:_

<img src="github_images/fsm.png" alt="FSM" width="600"/>

![Align right](github_images/align_right.gif)
![Align right](github_images/align_right_rviz.gif)

![Align left](github_images/align_left.gif)
![Align right](github_images/align_left_rviz.gif)

![LIDAR readings](github_images/lidarReadings_MAP_CENTER.json.png)
![RANSAC 1](github_images/Multiline_plot1.png)
![RANSAC 2](github_images/Multiline_plot2.png)

