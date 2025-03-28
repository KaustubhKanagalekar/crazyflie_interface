from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'backend', default_value='sim',
            description='cpp / cflib / sim backend for crazyflie (crazyswarm2)'
        ),
        DeclareLaunchArgument(
            'uri', default_value='',
            description='Robot number'
        ),
        Node(
            package='crazyflie_interface',
            executable='lqr_controller.py',
            name='lqr_controller',
            output='screen',
        ),
        Node(
            package='crazyflie_interface',
            executable='cbf.py',
            name = 'cbf_controller',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('crazyflie_interface'),
                    'launch',
                    'launch.py'
                ]),
            ]),
            launch_arguments={
                'backend': LaunchConfiguration('backend'),
                'uri': LaunchConfiguration('uri')
            }.items()
        ) 
    ])