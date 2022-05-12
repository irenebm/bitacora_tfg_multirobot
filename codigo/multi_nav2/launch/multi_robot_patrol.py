# Copyright 2021 Irene Bandera Moreno
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
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01}]


    # namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static'),
    #               ('/odom', 'odom'),
    #               ('/poses', 'poses'),
    #               ('/my_marker_goal', 'my_marker_goal')]

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')


    # Define commands for launching the navigation instances
    # multi_robot_cmds = []
    # for robot in robots:

    #     start_robot_bt = Node(
    #     package='multi_nav2',
    #     executable='patrolling_main',
    #     output='screen',
    #     namespace=TextSubstitution(text=robot['name']),
    #     remappings=remappings)
    start_robot_1_bt = Node(
        package='multi_nav2',
        executable='patrolling_main',
        output='screen',
        namespace='robot1',
        parameters=[
                {"publisher_port": 1266.0,
                "server_port": 1267.0}])

    start_robot_2_bt = Node(
        package='multi_nav2',
        executable='patrolling_main',
        output='screen',
        namespace='robot2',
        parameters=[
                {"publisher_port": 1366.0,
                "server_port": 1367.0}])

    #     multi_robot_cmds.append(start_robot_bt)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to start gazebo, robots and simulations

    # for robot in multi_robot_cmds:
    #     ld.add_action(robot)
    ld.add_action(start_robot_1_bt)
    ld.add_action(start_robot_2_bt)

    return ld
