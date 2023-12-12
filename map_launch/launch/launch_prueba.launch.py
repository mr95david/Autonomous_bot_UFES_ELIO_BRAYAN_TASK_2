#!/usr/bin/env python3

# Launch encargado de realizar el lanzamiento del mapa

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Funciones para spawnear el robot en una posicion especifica
    launch_turtlebot3_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    #pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    launch_cartograph_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')
    launch_nav2_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    slam_tools_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    # Seccion agregada
    config_dir = os.path.join(get_package_share_directory('map_launch1'), 'config')

    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    rviz_config= os.path.join(config_dir,'tb3_nav.rviz')

    # Definir la posicion donde desea que se spawnee el robot
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
    use_sim_time_m = LaunchConfiguration('use_sim_time', default = 'True')
    slam = LaunchConfiguration('slam', default = 'True')
    x_pose = LaunchConfiguration('x_pose', default='0')
    y_pose = LaunchConfiguration('y_pose', default='8')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_turtlebot3_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_turtlebot3_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Seccion de ingreso de cartografo FUNCIONA
    init_cartografo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_cartograph_dir, 'cartographer.launch.py')
        ),
        launch_arguments = {
            'use_sim_time': use_sim_time
        }.items()
    )

    # Seccion de inclusion de launch NO FUNCIONA
    init_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_nav2_dir, 'tb3_simulation_launch.py')
        ),
        launch_arguments = {
            'slam': slam,
            'use_sim_time': use_sim_time_m
        }.items()
    )

    # Seccion de inclusion de launch NO FUNCIONA
    init_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_tools_dir, 'online_async_launch.py')
        )
    )

    # Seccion agregada
    init_nav_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_nav2_dir, 'bringup_launch.py')
        ),
        launch_arguments = {
            'map': "",
            'params_file': params_file
        }.items()
    )

    rviz=Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        arguments=['-d',rviz_config]
    )

    ld = LaunchDescription()

    # Declarar opciones de lanzamiento
    #ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    #ld.add_action(init_cartografo)
    ld.add_action(init_nav_2)
    ld.add_action(init_slam)
    ld.add_action(rviz)
    
    #ld.add_action(init_nav)
    
    
    #ld.add_action(nav_launch_file_bringup)

    return ld