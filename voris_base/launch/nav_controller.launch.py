from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map', description='Reference frame id'),
        DeclareLaunchArgument('duct_x', default_value='3.0', description='Duct position along the X-axis'),
        DeclareLaunchArgument('duct_y', default_value='1.5', description='Duct position along the Y-axis'),
        DeclareLaunchArgument('duct_z', default_value='-1.0', description='Duct position along the Z-axis'),
        DeclareLaunchArgument('delta', default_value='0.2', description='Distance between waypoints of the path'),
        DeclareLaunchArgument('k_depth', default_value='5.0', description='Proportional gain for the depth controller'),
        DeclareLaunchArgument('k_yaw', default_value='2.0', description='Proportional gain for the yaw controller'),
        DeclareLaunchArgument('side', default_value='2.0', description='Side length of the square'),
        DeclareLaunchArgument('radius', default_value='1.0', description='Radius of the circle or spiral'),
        DeclareLaunchArgument('dz', default_value='-1.0', description='Vertical step size of the spiral'),
        DeclareLaunchArgument('turns', default_value='4', description='Number of spiral'),
        DeclareLaunchArgument('depth', default_value='4', description='Depth of the serpetine path'),
        DeclareLaunchArgument('path_type', default_value='circle', description='shape of the trajectory'),
        DeclareLaunchArgument('namespace', default_value=['Nav'], description='Namespace'),

        Node(
            package='voris_base',
            executable='path_generator',
            name='path_generator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'frame_id': LaunchConfiguration('frame_id')},
                        {'delta': LaunchConfiguration('delta')},
                        {'side': LaunchConfiguration('side')},
                        {'radius': LaunchConfiguration('radius')},
                        {'dz': LaunchConfiguration('dz')},
                        {'turns': LaunchConfiguration('turns')},
                        {'depth': LaunchConfiguration('depth')},
                        {'path_type': LaunchConfiguration('path_type')}
                        ],
        ),

        Node(
            package='voris_base',
            executable='trajectory_controller',
            name='trajectory_controller',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'duct_x': LaunchConfiguration('duct_x')},
                        {'duct_y': LaunchConfiguration('duct_y')},
                        {'duct_z': LaunchConfiguration('duct_z')}
                        ],
        )
    ])