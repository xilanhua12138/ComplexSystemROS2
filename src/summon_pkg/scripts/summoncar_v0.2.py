#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
import time
import os
import subprocess
from nav_msgs.msg import Odometry
import threading

def normalize_angle(angle):
    res = angle
    while res > math.pi:
        res -= 2.0 * math.pi
    while res < -math.pi:
        res += 2.0 * math.pi
    return res



class VoiceControlledRobot(Node):
    def __init__(self):
        super().__init__('voice_controlled_robot')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])

        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'Using simulated time: {use_sim_time}')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.rotation_from_angle,
            10)

        self.voice_subscription = self.create_subscription(
            String,
            '/hearts/stt',
            self.voice_command_callback,
            10)
        
        self.angle_subscription = self.create_subscription(
            Float64,
            '/xf_angle',
            self.angle_callback,
            10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.direction_angle = None  # Direction angle from sound localization

        self.is_adjusting = False  # New flag to control whether the robot is adjusting

        self.reverse = 1
        self.tolerance = math.radians(1.5)  # Tolerance in radians
        self.odom_angular_scale_correction = 1.0
        
        self.turn_angle = 0
        self.current_angle = None
        self.last_angle = None
        self.is_moving = False

    def voice_command_callback(self, msg):
        if "过来" in msg.data and self.direction_angle is not None:

            if not self.is_adjusting:
                self.adjust_angle = self.direction_angle
                
                if self.adjust_angle < 180 or self.adjust_angle > 300:
                    self.reverse = -self.reverse
                    if self.adjust_angle > 300:
                        self.adjust_angle = 360 - self.adjust_angle
                else:
                    self.adjust_angle = 300 - self.adjust_angle
                
                self.adjust_angle = math.radians(self.adjust_angle)  # Convert to radians
                self.adjust_angle *= self.reverse

                #reinitialize
                self.turn_angle = 0
                self.is_adjusting = True
                self.reverse = 1
                self.current_angle = None
                self.last_angle = None


    def angle_callback(self, msg):
        if not self.is_adjusting:
            self.direction_angle = msg.data  # Update direction only if not moving or adjusting

    def rotation_from_angle(self, msg):
        if self.is_adjusting and not self.is_moving:
            error = self.adjust_angle - self.turn_angle
            if self.last_angle is None:
                self.last_angle = self.calculate_yaw_from_msg(msg)
                return 
            
            if abs(error) > self.tolerance:

                move_cmd = Twist()
                move_cmd.angular.z = math.copysign(0.1, error)  # Set a moderate rotation speed
                self.cmd_vel_pub.publish(move_cmd)
                
                self.current_angle = self.calculate_yaw_from_msg(msg)
                delta_angle = self.odom_angular_scale_correction * normalize_angle(self.current_angle - self.last_angle)

                self.turn_angle += delta_angle

                error = self.adjust_angle - self.turn_angle
                self.last_angle = self.current_angle

                self.get_logger().info(
                    'Adjust Angle: {}, Turned Angle: {}'.format(
                        math.degrees(self.adjust_angle), math.degrees(self.turn_angle)))
                return
            
            if abs(error) <= self.tolerance:
                self.cmd_vel_pub.publish(Twist())  # Stop moving after adjustment
                self.is_adjusting = False  # Reset adjusting flag
                self.is_moving = True
                process = subprocess.Popen(["ros2", "run", "lazer_tracker_pkg", "lazer_tracker.py"])
                thread = threading.Thread(target=self.process_finishedcallback, args=(process,))
                thread.start()

                return 
        else:
            return
    
    def calculate_yaw_from_msg(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.get_logger().info('Current yaw: {:.2f} radians'.format(yaw))
        return yaw
    
    def process_finishedcallback(self,process):
        process.wait()
        self.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    voice_controlled_robot = VoiceControlledRobot()
    rclpy.spin(voice_controlled_robot)


if __name__ == '__main__':
    main()

