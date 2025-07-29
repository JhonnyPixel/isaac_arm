
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class PandaJointPublisher(Node):
    def __init__(self):
        super().__init__('panda_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        timer_period = 1.0  # secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Panda joint publisher started')

    def timer_callback(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_msg.position = [0.0, -0.3, 0.0, -2.0, 0.0, 2.0, 0.8]
        self.publisher_.publish(joint_msg)
        self.get_logger().info('Published joint command')

def main(args=None):
    rclpy.init(args=args)
    node = PandaJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
