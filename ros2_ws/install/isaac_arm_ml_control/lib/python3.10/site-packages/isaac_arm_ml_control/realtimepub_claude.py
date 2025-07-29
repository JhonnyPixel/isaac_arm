import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer
import csv
import os
import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from scipy.spatial.transform import Rotation as R


def cinematic(self, x, y, z, o):
    # Parametri DH corretti per KUKA LBR iiwa 7 R800
    dh_params = np.array([
        [0,     0.340,  0,          0],        # Joint 1
        [0,     0,      -np.pi/2,   0],        # Joint 2  
        [0,     0.400,  np.pi/2,    0],        # Joint 3
        [0,     0,      np.pi/2,    0],        # Joint 4
        [0,     0.400,  -np.pi/2,   0],        # Joint 5
        [0,     0,      -np.pi/2,   0],        # Joint 6
        [0,     0.126,  np.pi/2,    0]         # Joint 7
    ])

    # Creazione del robot
    robot = RobotSerial(dh_params)

    # Debug: stampa le coordinate ricevute
    self.get_logger().info(f"Target position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    # Posizione target
    xyz = np.array([[x], [y], [z]])
    
    # Conversione quaternione a euler
    r = R.from_quat([o.x, o.y, o.z, o.w])
    abc = r.as_euler('xyz', degrees=False)
    
    # Debug: stampa orientamento
    self.get_logger().info(f"Target orientation (euler): {abc}")
    
    # Creazione del frame desiderato
    end = Frame.from_euler_3(abc, xyz)

    # Calcolo della cinematica inversa
    robot.inverse(end)

    # Verifica se la soluzione è raggiungibile
    if robot.is_reachable_inverse:
        joint_positions = robot.axis_values.tolist()
        
        self.get_logger().info("Soluzione cinematica inversa trovata:")
        self.get_logger().info(f"Joint values: {[f'{j:.3f}' for j in joint_positions]}")

        # Pubblica comando joint
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]
        joint_msg.position = joint_positions

        self.publisher_.publish(joint_msg)
        
        return True
    else:
        self.get_logger().warn("La posizione desiderata non è raggiungibile con la cinematica inversa")
        return False


class VRDataLogger(Node):
    def __init__(self):
        super().__init__('vr_data_logger')

        # Message filters subscribers
        pose_sub = Subscriber(self, PoseStamped, 'vr/controller_pose')
        grip_sub = Subscriber(self, Float32, 'vr/trigger_pressure')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)

        # Synchronize by header timestamp / arrival time
        sync = ApproximateTimeSynchronizer(
            [pose_sub, grip_sub],
            queue_size=10,
            slop=0.01,
            allow_headerless=True
        )
        sync.registerCallback(self.synced_callback)
        
        # Aggiungi un offset per centrare il workspace
        self.workspace_offset = np.array([0.5, 0.0, 0.5])  # Regola questi valori
        self.scale_factor = 1.0  # Fattore di scala per i movimenti

        self.get_logger().info("VRDataLogger: pronto a ricevere pose VR sincronizzate")

    def synced_callback(self, pose_msg: PoseStamped, grip_msg: Float32):
        # Estrai pose
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation

        # Debug: stampa le coordinate VR originali
        self.get_logger().info(f"VR raw pos: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")

        # Trasformazione coordinate VR -> Robot
        # Testa diverse mappings per trovare quella corretta
        x_robot = p.z * self.scale_factor + self.workspace_offset[0]
        y_robot = -p.x * self.scale_factor + self.workspace_offset[1] 
        z_robot = p.y * self.scale_factor + self.workspace_offset[2]
        
        # Verifica limiti workspace (valori tipici per KUKA iiwa)
        # Workspace approssimativo: raggio ~0.8m
        if np.sqrt(x_robot**2 + y_robot**2 + z_robot**2) > 0.8:
            self.get_logger().warn(f"Target fuori workspace: {x_robot:.3f}, {y_robot:.3f}, {z_robot:.3f}")
            return

        # Debug: stampa coordinate robot
        self.get_logger().info(f"Robot target: x={x_robot:.3f}, y={y_robot:.3f}, z={z_robot:.3f}")

        # Calcola cinematica inversa e pubblica joints
        cinematic(self, x_robot, y_robot, z_robot, o)

    def destroy_node(self):
        self.get_logger().info("Nodo terminato")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
