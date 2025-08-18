from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the default launch file for Gazebo Sim
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_file = '/home/mahe/Desktop/hk/erc_hackathon_main/Automation/models/warehouse.sdf'
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world_file}.items()
    )

    return LaunchDescription([
        # Set the Gazebo resource path to include your models
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', 
            '/home/mahe/Desktop/hk/erc_hackathon_main/Automation/models'
        ),
        
        # Set Gazebo physics parameters for real-time performance
        SetEnvironmentVariable('GAZEBO_REALTIME_FACTOR', '1.0'),  # 1.0 means real-time speed

        # Launch the Gazebo simulation
        gazebo_launch
    ])
