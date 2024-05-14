#!/usr/bin/env python3
################################################################################
# File name  : chat.launch.py
# Author     : Derek Ripper
# Created    : 23 Feb 2022
#
# Purpose: To launch chat.
################################################################################
# # Updates:
# ??/???/???? by ????? -
#
################################################################################
from launch             import LaunchDescription
from launch_ros.actions import Node
from launch.actions     import ExecuteProcess

def generate_launch_description():
    ld =  LaunchDescription ([
        # Node(
        # package    ="hri_stt_pkg",
        # executable ="stt_v2.py",
        # name       ="stt_node", #Takes priorty over node name in package code
        # output     ="screen",
        # emulate_tty = True,
        # parameters =[
        # {"SR_SPEECH_ENGINE"    : "google"},
        # {"SR_ENERGY_THRESHOLD" : 300     },
        # {"SR_PAUSE_THRESHOLD"  : 1.01    },
        # ]),



        Node(
        package    ="chat_pkg",
        executable ="chat.py",
        name       ="chat_node", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True),

        Node(
        package    ="hri_tts_pkg",
        executable ="tts_v2.py",
        name       ="tts_node", #Takes priorty over node name in package code
        output     ="screen",
        emulate_tty = True),

        ])
    return ld
