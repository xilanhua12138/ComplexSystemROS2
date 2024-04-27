import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf_transformations
class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.yaw = None
        
        self.print_yaw()
        
    def odom_callback(self, msg):
        # 这里简单地打印出接收到的位置和姿态信息
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # self.get_logger().info(f"Position: {position}")
        # self.get_logger().info(f"Orientation: {yaw}")
        self.yaw = yaw

    def print_yaw(self):
        while True:
            self.get_logger().info(f"Orientation: {self.yaw}")

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    # 销毁节点，回收资源
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
