import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yesense_imu',
            executable='yesense_imu_node',
            name='yesense_imu_node',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', launch.substitutions.PathJoinSubstitution([
                launch_ros.substitutions.FindPackageShare('yesense_imu'),
                'rviz',
                'yesense.rviz'
            ])],
            output='screen'
        ),
    ])
