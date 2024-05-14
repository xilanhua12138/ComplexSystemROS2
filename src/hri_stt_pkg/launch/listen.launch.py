# Name  : listen.launch.py
# Author: Derek Ripper
# Date  : 12 Jan 2022
#
# Purpose: To listen for a range of topics
###################################################################################################
from launch             import LaunchDescription
from launch_ros.actions import Node
from launch.actions     import ExecuteProcess

def generate_launch_description():
    ld =  LaunchDescription ([
        Node(
        package    ="hri_stt_pkg",
        executable ="listen_v2.py",
        name       ="listen_node", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True,
        )
    ])
    return ld
