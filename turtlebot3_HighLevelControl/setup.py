from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_HighLevelControl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # Michele/Fabio: line to add to see launch file!
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # Michele/Fabio: line to add to see rviz config files!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='trotti.francescovr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_HighLevelControl = turtlebot3_HighLevelControl.highLevelControl:main',
            'turtlebot3_HighLevelControl_Exercise1a_onlyLeftAlign = turtlebot3_HighLevelControl.highLevelControl_Exercise1a_onlyLeftAlign:main',
            'turtlebot3_HighLevelControl_Exercise1b_right_or_left = turtlebot3_HighLevelControl.highLevelControl_Exercise1b_right_or_left:main',
            'turtlebot3_HighLevelControl_Exercise1c_rewind = turtlebot3_HighLevelControl.highLevelControl_Exercise1c_rewind:main',
        ],
    },
)
