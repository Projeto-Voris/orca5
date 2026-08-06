#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    orca_bringup_dir = get_package_share_directory('orca_bringup')
    voris_bringup_dir = get_package_share_directory('voris_bringup')

    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub_heavy.parm')
    mavros_params_file = os.path.join(orca_bringup_dir, 'param', 'sim_mavros_params.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'rviz', 'sim_heavy.rviz')
    world_file = os.path.join(orca_bringup_dir, 'worlds', 'inpetu_heavy.world')
    
    return LaunchDescription([
        DeclareLaunchArgument('ardusub', default_value='True', description='Launch ArduSUB with SIM_JSON?'),
        DeclareLaunchArgument( 'gzclient', default_value='False', description='Launch Gazebo UI?'),
        DeclareLaunchArgument( 'mavros', default_value='True', description='Launch mavros?'),
        DeclareLaunchArgument('description', default_value='True', description='Publish robot_description?'),
        DeclareLaunchArgument('rviz', default_value='True', description='Launch rviz?'),
       
       # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # 1. Define the ArduSub process
        ExecuteProcess(
            cmd=['ardusub', '-w', '-M', 'JSON','--defaults', ardusub_params_file,
                '-I0','--home', '-27.430278, -48.443520, 0.0, 0'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),

        ),

        # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['gz', 'sim', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '3',  '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Translate messages MAV <-> ROS
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            # mavros_node is actually many nodes, so we can't override the name
            # name='mavros_node',
            parameters=[mavros_params_file],
            condition=IfCondition(LaunchConfiguration('mavros')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([
                FindPackageShare('voris_description'), 'launch', 'voris_visualize.launch.py'])]),
            condition=IfCondition(LaunchConfiguration('description')),
        ),
])
