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
        package    ="hri_stt_pkg",
        executable ="stt_v2.py",
        name       ="stt_node", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True,
        parameters =[
        {"SR_SPEECH_ENGINE"    : "google"},
        {"SR_ENERGY_THRESHOLD" : 300     },
        {"SR_PAUSE_THRESHOLD"  : 0.8     },
        ]),

        # Node(
        # package    ="chat_pkg",
        # executable ="chat.py",
        # name       ="chat_node", #Takes priorty over node name in package code
        # output     ="screen",
        # emulate_tty = True),

        Node(
        package    ="hri_tts_pkg",
        executable ="tts_v2.py",
        name       ="tts_node", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True),

        ])
    return ld
