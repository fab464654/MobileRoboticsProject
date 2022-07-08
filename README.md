# MobileRoboticsProject

## _How to install the packages in your ROS2 workspace:_
- Refer to the official documentation to install ROS2 Foxy (https://docs.ros.org/en/foxy/Installation.html)
- Copy the "robust_wall_follower", "turtlebot3_description", "turtlebot3_visualizer" packages in your ROS2 workspace (src folder)
- Build and source the workspace
```sh
cd colcon_ws/
colcon build && . install/setup.bash
```

### _Simulation with Unity_
## _To tune the code (optional):_
Modify "robust_wall_follower/robust_wall_follower/robust_wall_follower.py" choosing:

### _SUGGESTED TUNABLE VALUES FOR SIMULATION:_
```sh
- distance_threshold = 0.15   # [m] distance under which the robot detects a wall/obstacle
- front_angle_half = 80       # [°] half width of front region, used to detect and find walls in the surroundings 
- focus_angle_half = 20       # [°] angle width of the front region used in 'align left/right' state
- side_angle = 20             # [°] width of lateral regions 'left' and 'right'  
- ransac_threshold = 0.001    # distance under which a point gives its consensus to a line 
- ransac_iterations = 200     # number of iterations of RANSAC algorithm  
- add_noise = True            # choose whether to add white noise to lidar readings 
- sigma = 0.005               # standard deviation of the optional added white noise 
- K_P = 1                     # controller P term 
- align_max_ang_vel = 0.5     # [rad/s] < 1.82! Maximum angular velocity in 'align left/right' state (saturation) 
```
 
 
 
 
 
 
 
Build and source the workspace


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

### _Real Turtlebot3 Burger robot_

## _To tune the code (optional):_
Modify "robust_wall_follower/robust_wall_follower/robust_wall_follower.py" choosing:

### _SUGGESTED TUNABLE VALUES FOR REAL ROBOT:_
```sh
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
```  
Build and source the workspace


## _How to run it (real Turtlebot3 bringup):_
- Modify "~/.bashrc" file specifying the robot ID and type; in our case:
```sh
export ROS_DOMAIN_ID=34             #TURTLEBOT3; number on the label on the robot!
export TURTLEBOT3_MODEL=burger      #"burger" or "waffle_pi" according to which robot you use
```
- Source "~/.bashrc" file
```sh
source ~/.bashrc
```
- Check that ROS_DOMAIN_ID has been set
```sh
echo $ROS_DOMAIN_ID 
```

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
The aim of this project was implementing a robust method for moving precisely aligned to a certain wall. In particular, it was an extention of the "High level control - 1.c" already implemented during laboratory session (see "turtlebot3_HighLevelControl/turtlebot3_HighLevelControl/highLevelControl_Exercise1c_rewind.py" file). What can be clearly seen testing the standard wall follower (both in simulation and on the real robot) is that during the "Follow wall" state, the robot continuously oscillates trying to move straight. This behaviour was obtained imposing an angular velocity that causes the robot to turn towards the wall, avoiding to go away from it. The non-perfect alignment is due to a basic algorithm to perform the task. In that case, the change of state inside the finite state machine depends on the pre-defined regions' amplitudes, as shown by the following picture.

<p align="center">
  <img src="github_images/regions.jpg" alt="regions" width="300"/>
  <img src="github_images/robot.gif" alt="turtlebot" width="400"/>
</p>


In the following image, the implemented Finite State Machine is shown. The algorithm has the ability to perform alignment with respect to the left or right wall. Also, when a key is pressed during the execution, the robot will turn by 180° ("Rotate 180°" state) and enter the "Rewind" state. This state makes the robot "going back" re-running a certain number of linear/angular velocities according to the values stored in a deque stack.
<p align="center">
  <img src="github_images/fsm.png" alt="FSM" width="500"/>
</p>

Some other problems that arise on the real robot are worth mentioning:
- the LIDAR readings are not always in the same quantity. In fact, there's some variability on the length of the structure that contains the measured distances.
- the LIDAR sensor doesn't return 360 values but far less (around 230) each second; moreover the array structure containing the measurements is NOT filled with placeholders that indicate the missing values. This makes the implementation harder, if it relies on regions identified by specific indeces.
- even if the "arena" that we build to contain the robot is taller than the robot itself, several readings refer to outside the walls. In particular, we suppose that this low-cost sensor points the laser "on an angle" with respect to the ground. In fact, the closest measurements are pretty reliable while the distant ones refer to outside the built arena.

## _Our solution:_
This project aims at implementing a robust algorithm to perfectly align to the right/left wall and then go straight until another wall is reached or until a key is pressed (switching to the "Rotate 180 deg" state). This behaviour wasn't achieved by the starting algorithm developed during the lab sessions, as can be clearly seen from the following video (Rviz2 + Unity visualization; the colors of the LIDAR readings match the previously shown regions).

<p align="center">
  <img src="github_images/rewind_lab_align_right.gif" alt="rewind LAB align right" width="550"/>
</p>






#### Align "Right" argument; Real robot on the left and real-time Rviz2 visualization on the right
<p align="center">
  <img src="github_images/align_right.gif" alt="Align right" width="400"/>
  <img src="github_images/align_right_rviz.gif" alt="Align right RVIZ" width="400"/>
</p>

#### Align "Left" argument; Real robot on the left and real-time Rviz2 visualization on the right
<p align="center">
  <img src="github_images/align_left.gif" alt="Align right" width="400"/>
  <img src="github_images/align_left_rviz.gif" alt="Align right RVIZ" width="400"/>
</p>

#### Align "Left" argument; Real robot test from above (2x speed)
https://user-images.githubusercontent.com/76775232/177985766-7f591c56-e5a5-4d4e-b149-81ba3bf3fcad.mp4



<p align="center">
  <img src="github_images/lidarReadings_MAP_CENTER.json.png" alt="LIDAR readings" width="800"/>
</p>

<p align="center">
  <img src="github_images/Multiline_plot1.png" alt="Multiline_plot1" width="800"/>
  <img src="github_images/Multiline_plot2.png" alt="Multiline_plot1" width="800"/>
</p>

