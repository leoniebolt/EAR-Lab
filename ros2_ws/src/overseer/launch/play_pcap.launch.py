import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Arguments (We will pass the path from the Master script)
    pcap_arg = DeclareLaunchArgument(
        'pcap_path',
        description='Absolute path to the pcap file'
    )
    
    # 2. Configuration
    pcap_path = LaunchConfiguration('pcap_path')
    calibration_file = os.path.join(
        get_package_share_directory('velodyne_pointcloud'), 
        'params', 'VeloView-VLP-32C.yaml' # Change to VLP-16.yaml or similar if using a Puck
    )

    return LaunchDescription([
        pcap_arg,

        # Driver Node (Reads PCAP -> Packets)
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[{
                'pcap': pcap_path,
                'model': '32C',      # Update this based on your exact sensor model
                'read_once': True,
                'read_fast': False,
                'rpm': 600.0,
                'frame_id': 'velodyne'
            }]
        ),

        # Transform Node (Packets -> PointCloud2)
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[{
                'model': '32C',
                'calibration': calibration_file,
                'min_range': 0.9,
                'max_range': 130.0,
                'organize_cloud': False,
            }],
            # Remap the output topic if needed. 
            # LIO-SAM expects 'points_raw', Velodyne outputs 'velodyne_points'
            remappings=[('/velodyne_points', '/points_raw')]
        )
    ])