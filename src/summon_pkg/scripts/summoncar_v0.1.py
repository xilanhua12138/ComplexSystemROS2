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
        self.is_moving = False
        self.is_adjusting = False  # New flag to control whether the robot is adjusting

        self.reverse = 1
        self.tolerance = math.radians(1.5)  # Tolerance in radians
        self.odom_angular_scale_correction = 1.0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'
        self.done =False
         
    def voice_command_callback(self, msg):
        if "过来" in msg.data and self.direction_angle is not None:
            self.is_adjusting = True  # Set adjusting flag
            self.done = False
            self.adjust_to_sound(self.direction_angle)  # First adjust orientation
            self.is_moving = True  # Set moving flag after adjustment

    def angle_callback(self, msg):
        if not self.is_adjusting and not self.is_moving:
            self.direction_angle = msg.data  # Update direction only if not moving or adjusting

    def adjust_to_sound(self, adjust_angle):
        if adjust_angle < 180 or adjust_angle > 300:
            self.reverse = -self.reverse
            if adjust_angle > 300:
                adjust_angle = 360 - adjust_angle
        else:
            adjust_angle = 300 - adjust_angle
        
        adjust_angle = math.radians(adjust_angle)  # Convert to radians
        adjust_angle *= self.reverse

        turn_angle = 0
        error = adjust_angle - turn_angle

        move_cmd = Twist()
        move_cmd.angular.z = math.copysign(0.1, error)  # Set a moderate rotation speed
        self.cmd_vel_pub.publish(move_cmd)

        time.sleep(0.2)

        # turn_angle = 0
        # last_angle = self.get_odom_angle()
        # error = adjust_angle - turn_angle

        # while abs(error) > self.tolerance:
        #     start_time = self.get_clock().now()
        #     move_cmd = Twist()
        #     move_cmd.angular.z = math.copysign(0.1, error)  # Set a moderate rotation speed
        #     self.cmd_vel_pub.publish(move_cmd)
            
        #     time.sleep(0.2)

        #     current_angle = self.get_odom_angle()
        #     delta_angle = self.odom_angular_scale_correction * normalize_angle(current_angle - last_angle)
        #     turn_angle += delta_angle
        #     error = adjust_angle - turn_angle
        #     last_angle = current_angle
            
        #     end_time = self.get_clock().now()
        #     elapsed_time = end_time - start_time
        #     self.get_logger().info(
        #         'Elapsed Time: {}, Adjust Angle: {}, Turned Angle: {}'.format(
        #             elapsed_time.nanoseconds, math.degrees(adjust_angle), math.degrees(turn_angle)))

        self.cmd_vel_pub.publish(Twist())  # Stop moving after adjustment
        self.is_adjusting = False  # Reset adjusting flag
        # self.move_towards_sound()  # Begin moving towards the sound after adjusting.

        # os.system("ros2 run lazer_tracker_pkg lazer_tracker.py")
        process = subprocess.Popen(["ros2", "run", "lazer_tracker_pkg", "lazer_tracker.py"])
        return 
    
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, self.get_clock().now())
        except Exception as e:
            self.get_logger().info(f"TF Exception: {e}")
            return

        # Convert the rotation from a quaternion to an Euler angle
        quat = trans.transform.rotation
        roll, pitch, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw
    
def main(args=None):
    rclpy.init(args=args)
    voice_controlled_robot = VoiceControlledRobot()
    rclpy.spin(voice_controlled_robot)


if __name__ == '__main__':
    main()

