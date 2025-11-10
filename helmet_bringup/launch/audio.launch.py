#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch ReSpeaker audio node"""
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='hw:ArrayUAC10,0',
        description='Audio device for ReSpeaker'
    )
    
    channels_arg = DeclareLaunchArgument(
        'channels',
        default_value='4',
        description='Number of audio channels'
    )
    
    # Get launch configurations
    device = LaunchConfiguration('device')
    channels = LaunchConfiguration('channels')
    
    # Get package directories
    helmet_bringup_dir = FindPackageShare('helmet_bringup')
    
    # Audio capture node
    audio_capture_node = Node(
        package='audio_capture',
        executable='audio_capture',
        name='audio_capture_node',
        parameters=[
            PathJoinSubstitution([
                helmet_bringup_dir,
                'config',
                'audio_params.yaml'
            ]),
            {
                'device': device,
                'channels': channels,
            }
        ],
        output='screen'
    )
    
    # ReSpeaker processing node (if available)
    respeaker_node = Node(
        package='respeaker_ros',
        executable='respeaker_node',
        name='respeaker_node',
        parameters=[
            PathJoinSubstitution([
                helmet_bringup_dir,
                'config',
                'respeaker_params.yaml'
            ])
        ],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        channels_arg,
        audio_capture_node,
        respeaker_node,
    ])