# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_waffle_pi'
    urdf_path = os.path.join(
        get_package_share_directory('tb3_worlds'),
        'models',
        model_folder,
        'model.sdf'
    )
    
    bridge_params = os.path.join(
    get_package_share_directory('tb3_worlds'),
    'params',
    'waffle_bridge.yaml'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')


    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')
    
    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_ros_gz_sim_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )
    
    start_gazebo_ros_bridge_cmd = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '--ros-args',
        '-p',
        # f'config_file:={bridge_params}',
        'config_file:=./src/tb3_worlds/params/waffle_bridge.yaml'
    ],
    output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(declare_x_position_cmd)
    # ld.add_action(declare_y_position_cmd)
    # ld.add_action(declare_z_position_cmd)


    # Add any conditioned actions
    ld.add_action(start_ros_gz_sim_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)

    return ld
