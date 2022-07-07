# MobileRoboticsProject

## _How to use it:_
```sh
ros2 run ...
```

## _About the problem:_
The aim of this project was implementing a robust method for moving parallel to a certain wall. In particular, it was an extention of the "High level control - 1.c" already implemented during laboratory session. What can be clearly seen testing the standard wall follower (both in simulation and on the real robot) is that during the "Follow wall" state, the robot continuously oscillates trying to move straight. This behaviour was obtained imposing an angular velocity that causes the robot to turn towards the wall, avoiding to go away from it. The non-perfect alignment is due to odometry/LIDAR readings that are affected by noise and errors. 

In the following image, the implemented Finite State Machine is shown. The algorithm has the ability to perform alignment towards left or right. Also, when a key is pressed during the execution, the robot will turn by 180° ("Rotate 180°" state) and enter the "Rewind" state. This state makes the robot "going back" re-running a certain number of linear/angular velocities according to the values stored in a deque stack.

## _Our solution:_



![LIDAR readings](github_images/lidarReadings_MAP_CENTER.json.png)
![RANSAC 1](github_images/Multiline_plot1.png)
![RANSAC 2](github_images/Multiline_plot2.png)

