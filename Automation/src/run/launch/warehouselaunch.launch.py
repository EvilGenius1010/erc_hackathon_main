import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    warehouse_world = '/home/mahe/Desktop/hk/erc_hackathon_main/Automation/models/warehouse.sdf'
    robot_sdf = '/home/mahe/Desktop/hk/erc_hackathon_main/Automation/models/robot.sdf'

    declare_robot = DeclareLaunchArgument(
        'robot_sdf',
        default_value=robot_sdf,
        description='Path to robot sdf file'
    )
    declare_world = DeclareLaunchArgument(
        'warehouse_world',
        default_value=warehouse_world,
        description='Path to Ignition Gazebo world file'
    )

    # Launch Ignition Gazebo with the world file
    ignite_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', warehouse_world, '-v', '4'],
        output='screen'
    )

    # Spawn robot using ros_ign_gazebo "create" executable node
    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-file', robot_sdf, '-name', 'forklift', '-allow_renaming', 'true'],
        output='screen'
    )

    return LaunchDescription([
        declare_robot,
        declare_world,
        ignite_sim,
        spawn_robot,
    ])
