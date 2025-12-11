#!/usr/bin/env python3
"""
Main launch file for Escape Maze Simulation
Launches all components together
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # ===== SIMULATION ENVIRONMENT =====
        # Launch Gazebo with maze world
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 os.path.join(get_package_share_directory('escape_maze'),
                             'worlds', 'escape_maze.world')],
            output='screen'
        ),
        
        # Spawn TurtleBot3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3_waffle',
                      '-topic', 'robot_description',
                      '-x', '0.0', '-y', '0.0', '-z', '0.01'],
            output='screen'
        ),
        
        # ===== NAVIGATION STACK =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),
        
        # ===== MAZE CONTROL SYSTEM =====
        # Main Controller
        Node(
            package='escape_maze',
            executable='maze_controller',
            name='maze_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Clue Detector
        Node(
            package='escape_maze',
            executable='clue_detector',
            name='clue_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Puzzle Logic Engine
        Node(
            package='escape_maze',
            executable='puzzle_logic_engine',
            name='puzzle_logic_engine',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Navigation Manager
        Node(
            package='escape_maze',
            executable='maze_navigation_manager',
            name='maze_navigation_manager',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # System Monitor
        Node(
            package='escape_maze',
            executable='system_monitor',
            name='system_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Voice Emergency Stop (simulation mode)
        Node(
            package='escape_maze',
            executable='voice_emergency_stop',
            name='voice_emergency_stop',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # ===== VISUALIZATION =====
        # RViz with maze configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('escape_maze'),
                                         'config', 'escape_maze.rviz')],
            output='screen'
        ),
        
        # ===== UTILITIES =====
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])
