#!/usr/bin/env python3

"""
Bring up SLAM nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    nodes = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time?',
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value=['orbslam3']
        ),

        DeclareLaunchArgument(
            'vocabulary',
            default_value=PathJoinSubstitution([
                FindPackageShare('orbslam3_ros2'),
                'vocabulary',  # Assuming your vocab file is in the vocabulary directory
                'ORBvoc.txt'   # Replace with your actual vocabulary file name
            ]),
            description='Path to the ORB_SLAM3 vocabulary file'
        ),
        DeclareLaunchArgument(
            'pangolin',
            default_value="False",
            description='Use the viewer'
        ),

        DeclareLaunchArgument(
            'bridge',
            default_value='True',
            description='Launch SLAM bridge?',
        ),

        DeclareLaunchArgument(
            'orb',
            default_value='False',
            description='Launch ORB_SLAM3?',
        ),

        DeclareLaunchArgument(
            'mav_device',
            default_value='udpin:0.0.0.0:14551',
            description='MAVLink device address',
        ),

        DeclareLaunchArgument(
            'use_vpe',
            default_value='True',
            description='Use VISION_POSITION_ESTIMATE instead of VISION_POSITION_DELTA?',
        ),

        DeclareLaunchArgument(
            'rescale',
            default_value='False',
            description='Rescale Image'
        ),

        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='base_link',
            description='Parent link of SLAM frame'
        ),

        DeclareLaunchArgument(
            'child_frame_id',
            default_value='left_camera_link',
            description='link of SLAM frame'
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='PointCloud SLAM link'
        ),

        DeclareLaunchArgument(
            'yaml_file',
            default_value='stereosim.yaml',
            description='Name of the ORB_SLAM3 YAML configuration file'
        ),

        DeclareLaunchArgument('left_image', default_value=['/left_image_raw'], description='stereo left image'),
        DeclareLaunchArgument('right_image', default_value=['/right_image_raw'], description='stereo right image'),
        DeclareLaunchArgument('voc_file', default_value='/home/biancarosa/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/vocabulary/ORBvoc.txt', 
                  description='Caminho para o vocabulário ORB'),
        DeclareLaunchArgument('settings_file', default_value='/home/biancarosa/ros2_ws/src/orbslam3_ros2/orbslam3_ros2/config/stereosim.yaml', 
                  description='Caminho para o settings .yaml'),
        DeclareLaunchArgument('tracked_points', default_value='True', description='Publish tracked points?'),
        DeclareLaunchArgument('pose', default_value='/camera_pose', description='Pose topic name'),

        Node(
            package='orbslam3_ros2',
            executable='stereo',
            name = 'stereo_orbslam3',
            output='screen',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'voc_file': LaunchConfiguration('voc_file'),
                'settings_file': LaunchConfiguration('settings_file'),
                'rescale': LaunchConfiguration('rescale'),
                'do_rectify': True,
                'ENU_publish': True,
                'parent_frame_id': LaunchConfiguration('parent_frame_id'),
                'child_frame_id': LaunchConfiguration('child_frame_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'tracked_points': LaunchConfiguration('tracked_points')
            }],
            remappings=[
                ('camera/left', LaunchConfiguration('left_image')),
                ('camera/right', LaunchConfiguration('right_image')),
                # ('/camera_pose/scaled', LaunchConfiguration('pose'))
            ],
            condition=IfCondition(LaunchConfiguration('orb')),
        ),
        # ExecuteProcess(
        #     cmd=['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher',
        #             # '--yaw', '-1.570796327',
        #             # '--roll', '-1.5707963270',
        #             # '--pitch', '0',
        #             '--frame-id', 'map',
        #             '--child-frame-id', 'slam'],
        #     output='screen',
        # ),

        Node(
            package='orca_bridge',
            executable='orbslam_bridge.py',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'mav_device': LaunchConfiguration('mav_device'),
                'use_vpe': LaunchConfiguration('use_vpe'),
            }],
            condition=IfCondition(LaunchConfiguration('bridge')),
        ),
    ]

    return LaunchDescription(nodes)
