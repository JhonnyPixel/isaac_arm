#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch
import torch.nn as nn
import numpy as np

# Definizione semplice rete MLP
class JointControlNet(nn.Module):
    def __init__(self, input_dim=8, output_dim=7):
        super().__init__()
        self.net = nn.Sequential(
                    nn.Linear(input_dim, 128),
                    nn.ReLU(),
                    nn.Linear(128, 128),
                    nn.ReLU(),
                    nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.net(x)

class PandaMLNode(Node):
    def __init__(self):
        super().__init__('iiwa_ml_node')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)
        timer_period = 0.2  # pubblica ogni 0.2 secondo
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Inizializza e carica rete
        self.model = JointControlNet()
        #decommenta quando hai il file con i pesi
        self.model.load_state_dict(torch.load('model_weights.pth'))
        self.model.eval()

        #carica i dati di input della sim. e crea un index per usarli
        data = np.load('training_data.npz')
        self.inputs = data['inputs']
        self.index = 0

        self.get_logger().info('Nodo ML iiwa avviato e rete caricata')

    def timer_callback(self):
        # Qui simuliamo input: 8 input
        #input_state = np.random.rand(8).astype(np.float32)

        #Qui uso i dati di input delle registrazioni
        if self.index >= len(self.inputs):
                self.index = 0
        input_state = self.inputs[self.index].astype(np.float32)
        self.index += 1
        #fine uso 
        
        input_tensor = torch.from_numpy(input_state).unsqueeze(0)  # batch 1

        # Inferenza rete
        with torch.no_grad():
            output = self.model(input_tensor)
        
        joint_positions = output.squeeze(0).numpy().tolist()

        # Pubblica comando joint
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
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
