#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from collections import deque

RAD2DEG = 180 / math.pi
class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

class LaserTracker(Node):
    def __init__(self):
        super().__init__('laser_tracker')
        self.declare_parameter('targetDist', 0.1)
        self.response_dist = self.get_parameter('targetDist').value
        self.moving = False
        self.switch = False
        self.lin_pid = SinglePID(1.0, 0.0, 1.5)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.laser_angle = 90
        self.priority_angle = 30

        self.sub_laser = self.create_subscription(
            LaserScan, '/scan', self.register_scan, qos_profile_sensor_data)
        
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.window_size = 100
        self.avg_velocity_x = deque(maxlen=self.window_size)

    def cancel(self):
        self.pub_vel.publish(Twist())
        self.sub_laser.destroy()
        self.get_logger().info("Shutting down this node.")

    def register_scan(self, scan_data):

        ranges = np.array(scan_data.ranges)
        offset = 0.5
        front_dist_list = []
        front_dist_id_list = []
        min_dist_list = []
        min_dist_id_list = []
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.priority_angle):
                if ranges[i] < (self.response_dist + offset):
                    front_dist_list.append(ranges[i])
                    front_dist_id_list.append(angle)
            elif (180 - self.laser_angle) < angle < (180 - self.priority_angle):
                min_dist_list.append(ranges[i])
                min_dist_id_list.append(angle)
            elif (self.priority_angle - 180) < angle < (self.laser_angle - 180):
                min_dist_list.append(ranges[i])
                min_dist_id_list.append(angle)

        if len(front_dist_id_list) != 0:
            min_dist = min(front_dist_list)
            min_dist_id = front_dist_id_list[front_dist_list.index(min_dist)]
        else:
            min_dist = min(min_dist_list)
            min_dist_id = min_dist_id_list[min_dist_list.index(min_dist)]
        
        # if self.moving:
        #     self.pub_vel.publish(Twist())
        #     self.moving = False
        #     return
        
        self.moving = True
        velocity = Twist()
        if abs(min_dist - self.response_dist) < 0.1: 
            min_dist = self.response_dist

        velocity.linear.x = -self.lin_pid.pid_compute(self.response_dist, min_dist)
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(min_dist_id)) / 72, 0)
        
        if min_dist_id > 0:
            velocity.angular.z = -ang_pid_compute 
        else:
            velocity.angular.z = ang_pid_compute
        
        if ang_pid_compute < 0.02: 
            velocity.angular.z = 0.0
        
        self.avg_velocity_x.append(velocity.linear.x)
        if len(self.avg_velocity_x) == self.window_size and sum(self.avg_velocity_x)/self.window_size < 0.02:
            self.pub_vel.publish(Twist())
            return 

        self.pub_vel.publish(velocity)
        return


def main(args=None):
    rclpy.init(args=args)
    tracker = LaserTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
