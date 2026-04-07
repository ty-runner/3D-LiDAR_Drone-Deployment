from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            parameters=[{'serial_baudrate': 460800}]
        ),

        # Servo control nodes
        Node(
            package='servo_control',
            executable='scan_sequence_node',
            name='scan_sequence_node'
        ),
        Node(
            package='servo_control',
            executable='servo_motor_node.py',
            name='servo_motor_node'
        ),
        Node(
            package='servo_control',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster'
        ),
        Node(
            package='servo_control',
            executable='lidar_pointcloud_node',
            name='lidar_pointcloud_node'
        ),

        # Topic throttle node
        '''Node(
            package='topic_tools',
            executable='throttle',
            name='throttle_lidar',
            arguments=['messages', '/lidar_points', '5.0', '/lidar_points_throttled']
        ),'''

        # Octomap server
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap',
            remappings=[('cloud_in', '/lidar_points_throttled')],
            parameters=[
                {'queue_size': 1000},
                {'resolution': 0.1},
                {'sensor_model/max_range': 6.0},
                {'sensor_model/hit': 0.7},
                {'sensor_model/miss': 0.4},
                {'frame_id': 'base_link'},
                {'transform_tolerance': 0.5}
            ]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])

