#<?xml version="1.0" encoding="UTF-8"?> 
#<launch>

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory("icars_graph")
    param_file = LaunchConfiguration("parameter_file")
    #rviz_config_file = os.path.join(share_dir, "config", "rviz2.rviz")

    #params = DeclareLaunchArgument("parameter_file",default_value=os.path.join(share_dir, "config"))


    return LaunchDescription([
       # params,

        Node(package="icars_graph", executable="icars_graph_test", name="icars_graph_test", parameters=[param_file], output="screen"),
        Node(package="icars_graph", executable="icars_graph_optimization", name="icars_graph_optimization", parameters=[param_file], output="screen"),
        Node(package="icars_graph", executable="icars_graph_imuIntegration", name="icars_graph_imuIntegration", parameters=[param_file], output="screen")])