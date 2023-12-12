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
    nodo_detecion = Node(
        package='work_node',
        executable='n_detection',
        output='screen',
    )

    nodo_local = Node(
        package='work_node',
        executable='n_localization',
        output='screen',
    )

    nodo_explo = Node(
        package='work_node',
        executable='n_exploration',
        output='screen',
    )
    
    ld = LaunchDescription()
    ld.add_action(nodo_detecion)
    ld.add_action(nodo_local)
    ld.add_action(nodo_explo)

    return ld