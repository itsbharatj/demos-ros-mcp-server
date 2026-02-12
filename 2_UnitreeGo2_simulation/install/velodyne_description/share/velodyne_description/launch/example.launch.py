#!/usr/bin/env python3
# Copyright (c) 2015-2024, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted under the BSD license.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_velodyne_description = get_package_share_directory('velodyne_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Set Gazebo resource path to find meshes
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_velodyne_description, '..')
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    gpu = LaunchConfiguration('gpu')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_velodyne_description, 'world', 'example.sdf'),
        description='Full path to world SDF file'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )

    declare_gpu = DeclareLaunchArgument(
        'gpu',
        default_value='true',
        description='Use GPU acceleration for lidar'
    )

    # Robot description from xacro
    robot_description_content = Command([
        'xacro ',
        os.path.join(pkg_velodyne_description, 'urdf', 'example.urdf.xacro'),
        ' gpu:=', gpu
    ])

    robot_description = {'robot_description': robot_description_content}

    # Gazebo Fortress simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'velodyne_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Bridge for VLP-16 lidar
    bridge_vlp16 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_vlp16',
        arguments=[
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/lidar/points', '/velodyne_points'),
        ],
        output='screen'
    )

    # Bridge for HDL-32E lidar
    bridge_hdl32 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_hdl32',
        arguments=[
            '/lidar2/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/lidar2/points', '/velodyne_points2'),
        ],
        output='screen'
    )

    # Bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_velodyne_description, 'rviz', 'example_ros2.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )

    return LaunchDescription([
        # Set environment variable
        set_gazebo_resource_path,
        # Launch arguments
        declare_use_sim_time,
        declare_world,
        declare_rviz,
        declare_gpu,
        # Nodes
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge_clock,
        bridge_vlp16,
        bridge_hdl32,
        rviz2,
    ])
