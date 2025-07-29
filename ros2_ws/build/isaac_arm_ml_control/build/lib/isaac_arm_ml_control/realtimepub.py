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


def cinematic(self,x,y,z,o):

    # 1. Definizione dei parametri DH per il KUKA LBR iiwa 7 R800
    dh_params = np.array([
        [0.34, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.4, 0., -np.pi / 2, 0.],
        [0., 0.,  np.pi / 2, 0.],
        [0.126, 0., 0., 0.]
    ])

    # 2. Creazione del robot
    robot = RobotSerial(dh_params)

    # 3. Definizione della posa desiderata dell'end-effector
    xyz = np.array([[x], [y], [z]])  # Posizione in metri
    
    r = R.from_quat([o.x, o.y, o.z, o.w])
    abc = r.as_euler('xyz') # Orientamento in radianti (ZYX)
    # 4. Creazione del frame desiderato
    end = Frame.from_euler_3(abc, xyz)

    # 5. Calcolo della cinematica inversa
    robot.inverse(end)

    # 6. Verifica se la soluzione è raggiungibile
    if robot.is_reachable_inverse:
        print("Soluzione trovata:")
        print("Valori dei giunti:", robot.axis_values)

        # 7. Esportazione dei dati in CSV
        joint_positions = robot.axis_values.tolist()
        
         # Pubblica comando joint
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6', 'joint7'
        ]
        joint_msg.position = joint_positions

        self.publisher_.publish(joint_msg)
        
        self.get_logger().info(
            f"{joint_positions[0]},{joint_positions[1]},{joint_positions[2]},{joint_positions[3]},{joint_positions[4]},{joint_positions[5]},{joint_positions[6]}" 
        )
        
        return True

    else:
        print("La posizione desiderata non è raggiungibile.")
        return False


class VRDataLogger(Node):
    def __init__(self):
        super().__init__('vr_data_logger')


        # message_filters subscribers
        pose_sub = Subscriber(self, PoseStamped, 'vr/controller_pose')
        grip_sub = Subscriber(self, Float32, 'vr/trigger_pressure')
        self.publisher_ = self.create_publisher(JointState, '/joint_command', 10)

        # Synchronize by header timestamp / arrival time (allow headerless)
        sync = ApproximateTimeSynchronizer(
            [pose_sub, grip_sub],
            queue_size=10,
            slop=0.01,
            allow_headerless=True
        )
        sync.registerCallback(self.synced_callback)

        self.get_logger().info("VRDataLogger: pronti a ricevere pose e pressione trigger sincronizzate")

    def synced_callback(self, pose_msg: PoseStamped, grip_msg: Float32):
        # extract pose
        p = pose_msg.pose.position
        o = pose_msg.pose.orientation

        # extract grip
        grip = grip_msg.data

        # log to console without timestamp
        self.get_logger().info(
            f"pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) | "
            f"ori=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f}) | grip={grip:.3f}"
        )

        x_robot = p.z
        y_robot = -p.x
        z_robot = p.y
        
        
	    # cambiamo le coordinate in base al sistema di riferimento adottato da isaac sim Z su, X avanti.

        # calculate inverse kinematic and publish joints
        cinematic(self,x_robot,y_robot,z_robot,o)
           

    def destroy_node(self):
        # close CSV
        try:
            #self.csv_file.close()
            self.get_logger().info(f"File '{self.csv_filename}' chiuso.")
        except Exception as e:
            self.get_logger().error(f"Errore chiusura CSV: {e}")
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
