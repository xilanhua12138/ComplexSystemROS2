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
        package    ="hri_tts_pkg",
        executable ="tts_node.py",
        name       ="tts_node", 
        output     ="screen",
        emulate_tty = True),
        Node(
        package    ="chat_pkg",
        executable ="chat_node.py",
        name       ="chat_node", 
        output     ="screen",
        emulate_tty = True)
        ])
    return ld
