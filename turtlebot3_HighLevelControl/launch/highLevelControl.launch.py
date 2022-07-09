from http.server import executable
import os
#import sys
from sys import prefix
#import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue  # Need master or Galactic branch for this feature
import xacro



def generate_launch_description():

    package_name = "turtlebot3_HighLevelControl"
    package_dir = os.path.join(Path(os.getenv('COLCON_PREFIX_PATH')).parent.absolute(), "src", package_name) # in src
    rviz_config_file_name = "highLevelControl.rviz"

    path_to_urdf = get_package_share_path('turtlebot3_description') / 'urdf' / 'turtlebot3_burger.urdf'

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        #arguments=['-d', [os.path.join(get_package_share_directory(package_name), 'rviz', rviz_config_file_name)]], # this do not open the source .rviz file but a copy in install
        arguments=['-d', [os.path.join(package_dir, 'rviz', rviz_config_file_name)]],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf)]), value_type=str
            )
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher_node)

    return ld
