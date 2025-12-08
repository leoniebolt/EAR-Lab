import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. Dynamic Path Finding ---
    # We need to find the 'assets' folder relative to the workspace.
    # Since we are inside a container, hardcoding '/root/ROS2_WS/assets' 
    # is often safer, but here is a dynamic way:
    base_path = os.path.expanduser('~/ROS2_WS/assets') 
    
    # --- 2. Define Arguments ---
    # Change 'nya_03/capture.pcap' to your actual file structure inside assets
    dataset_arg = DeclareLaunchArgument(
        'dataset_file', 
        default_value='mimap_in_slam_00/mimap_in_slam_00_20200117_JXL_03_201.pcap', 
        description='Path to pcap relative to assets folder'
    )

    dataset_full_path = [base_path, '/', LaunchConfiguration('dataset_file')]

    # --- 3. Include the PCAP Player ---
    overseer_pkg = get_package_share_directory('overseer')
    
    pcap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(overseer_pkg, 'launch', 'play_pcap.launch.py')
        ),
        launch_arguments={'pcap_path': dataset_full_path}.items()
    )

    # --- 4. Include the SLAM (LIO-SAM) ---
    # Note: We assume LIO-SAM is built and sourced
    lio_sam_pkg = get_package_share_directory('lio_sam')
    
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lio_sam_pkg, 'launch', 'run.launch.py')
        ),
        # You can pass args to LIO-SAM here if its launch file supports them
        # launch_arguments={'some_param': 'value'}.items()
    )

    # --- 5. Return Description ---
    return LaunchDescription([
        dataset_arg,
        pcap_launch,
        slam_launch
    ])