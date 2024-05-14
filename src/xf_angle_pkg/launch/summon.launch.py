# Name  : chat.launch.py
# Author: Derek Ripper
# Date  : 23 Feb 2022
#
# Purpose: To launch chat.
###################################################################################################
from launch             import LaunchDescription
from launch_ros.actions import Node
from launch.actions     import ExecuteProcess
import warnings
warnings.filterwarnings('ignore')
def generate_launch_description():
    ld =  LaunchDescription ([
        Node(
        package    ="summon_pkg",
        executable ="summoncar_v0.2.py",
        name       ="summon_node", 
        output     ="screen",
        emulate_tty = True,
        parameters=[{'use_sim_time': True}]),
        ])
    return ld
