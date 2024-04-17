# Name  : stt.launch.py
# Author: Derek Ripper
# Date  : 12 Jan 2022
#
# Purpose: To lauch Speech to text node
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
        {"SR_ENERGY_THRESHOLD" : 650     },
        {"SR_PAUSE_THRESHOLD"  : 0.80    },
        {"SR_MIC_VOLUME"       : 80      }, # % for microphone volume
        ])
    ])
    return ld
