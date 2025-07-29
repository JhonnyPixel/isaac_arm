#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch
import torch.nn as nn
import numpy as np

# Definizione semplice rete MLP
class JointControlNet(nn.Module):
    def __init__(self, input_dim=14, output_dim=7):
        super().__init__()
        self.model = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, output_dim)
        )

    def forward(self, x):
        return self.model(x)

class PandaMLNode(Node):
    def __init__(self):
        super().__init__('panda_ml_node')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        timer_period = 1.0  # pubblica ogni secondo
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Inizializza e carica rete
        self.model = JointControlNet()
        #decommenta quando hai il file con i pesi
        #self.model.load_state_dict(torch.load('model_weights.pth'))
        self.model.eval()

        self.get_logger().info('Nodo ML Panda avviato e rete caricata')

    def timer_callback(self):
        # Qui simuliamo input: 7 posizioni + 7 velocità random
        input_state = np.random.rand(14).astype(np.float32)
        input_tensor = torch.from_numpy(input_state).unsqueeze(0)  # batch 1

        # Inferenza rete
        with torch.no_grad():
            output = self.model(input_tensor)
        
        joint_positions = output.squeeze(0).numpy().tolist()

        # Pubblica comando joint
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        joint_msg.position = joint_positions

        self.publisher_.publish(joint_msg)
        self.get_logger().info(f'Comando pubblicato: {joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = PandaMLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
