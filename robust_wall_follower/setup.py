from setuptools import setup
import os
from glob import glob

package_name = 'robust_wall_follower'
functions_folder = 'robust_wall_follower/functions'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, functions_folder],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # Michele/Fabio: line to add to see launch file!
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # Michele/Fabio: line to add to see rviz config files!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michele',
    maintainer_email='michele@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robust_wall_follower = robust_wall_follower.robust_wall_follower:main'
        ],
    },
)
