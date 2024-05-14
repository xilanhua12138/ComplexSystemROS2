# Name  : chat.launch.py
# Author: Derek Ripper
# Date  : 23 Feb 2022
#
# Purpose: To launch chat.
###################################################################################################
from launch             import LaunchDescription
from launch_ros.actions import Node
from launch.actions     import ExecuteProcess

def generate_launch_description():
    ld =  LaunchDescription ([
        Node(
        package    ="xf_angle_pkg",
        executable ="xf_angle.py",
        name       ="xf_angle", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True)
        ])
    return ld
