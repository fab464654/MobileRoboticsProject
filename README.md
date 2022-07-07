# MobileRoboticsProject

## _How to install the packages in your ROS2 workspace:_
- Refer to the official documentation to install ROS2 Foxy (https://docs.ros.org/en/foxy/Installation.html)
- Copy the "robust_wall_follower", "turtlebot3_description", "turtlebot3_visualizer" packages in your ROS2 workspace (src folder)
- Build and source the workspace
```sh
cd colcon_ws/
colcon build && . install/setup.bash
```

# _Simulation with Unity_
## _To tune the code (optional):_
> Modify "robust_wall_follower/robust_wall_follower/robust_wall_follower.py" choosing:

> _SUGGESTED TUNABLE VALUES FOR SIMULATION:_
- distance_threshold = 0.15   # [m] distance under which the robot detects a wall/obstacle
- front_angle_half = 80       # [°] half width of front region, used to detect and find walls in the surroundings 
- focus_angle_half = 20       # [°] angle width of the front region used in 'align left/right' state
- side_angle = 20             # [°] width of lateral regions 'left' and 'right'  
- ransac_threshold = 0.001    # distance under which a point gives its consensus to a line 
- ransac_iterations = 200     # number of iterations of RANSAC algorithm  
- add_noise = True            # choose whether to add white noise to lidar readings 
- sigma = 0.005                # standard deviation of the optional added white noise 
- K_P = 1                     # controller P term 
- align_max_ang_vel = 0.5     # [rad/s] < 1.82! Maximum angular velocity in 'align left/right' state (saturation) 
    
> Build and source the workspace


## _How to run it (simulation with Unity):_
- Run Unity and click on the play button (using the "Turtlebot3UnityROS2" map)
- Run the launch file to start Rviz2:
```sh
$ ros2 launch robust_wall_follower robust_wall_follower.launch.py
```
- Run the implemented ROS2 node (choosing the desired align_direction left/right):
```sh
ros2 run robust_wall_follower robust_wall_follower --ros-args -p align_direction:=left
```


# _Real Turtlebot3 Burger robot_

## _To tune the code (optional):_
> Modify "robust_wall_follower/robust_wall_follower/robust_wall_follower.py" choosing:

> _SUGGESTED TUNABLE VALUES FOR REAL ROBOT:_
- distance_threshold = 0.25   # [m] distance under which the robot detects a wall/obstacle
- front_angle_half = 80       # [°] half width of front region, used to detect and find walls in the surroundings 
- focus_angle_half = 20       # [°] angle width of the front region used in 'align left/right' state
- side_angle = 20             # [°] width of lateral regions 'left' and 'right'  
- ransac_threshold = 0.001    # distance under which a point gives its consensus to a line 
- ransac_iterations = 200     # number of iterations of RANSAC algorithm  
- add_noise = False           # choose whether to add white noise to lidar readings (NO need to add noise on the real lidar)
- sigma = 0.005               # standard deviation of the optional added white noise (NOT USED)
- K_P = 1                     # controller P term 
- align_max_ang_vel = 0.5     # [rad/s] < 1.82! Maximum angular velocity in 'align left/right' state (saturation) 
    
> Build and source the workspace


## _How to run it (real Turtlebot3 bringup):_
- Refer to the official website to perform the bringup (https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/)
- Run the launch file to start Rviz2:
```sh
$ ros2 launch robust_wall_follower robust_wall_follower.launch.py
```
- Run the implemented ROS2 node (choosing the desired align_direction left/right):
```sh
ros2 run robust_wall_follower robust_wall_follower --ros-args -p align_direction:=left
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

