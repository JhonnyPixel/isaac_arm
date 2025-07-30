#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import openvr
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R
import numpy as np

class VRControllerPublisher(Node):
    def __init__(self):
        super().__init__('vr_controller_publisher')
        # Publisher for controller pose
        self.publisher_ = self.create_publisher(PoseStamped, 'vr/controller_pose', 10)
        self.trigger_pub_ = self.create_publisher(Float32, 'vr/trigger_pressure', 10)
        # Initialize OpenVR
        openvr.init(openvr.VRApplication_Other)
        self.vrsystem = openvr.VRSystem()
        # Choose left or right controller; here we use right hand
        self.controller_index = self._find_controller_index(openvr.TrackedControllerRole_RightHand)
        self.timer = self.create_timer(1, self.timer_callback)  # 50 Hz
        self.get_logger().info(f"Publishing VR controller poses on 'vr/controller_pose'. Controller index: {self.controller_index}")

    def _find_controller_index(self, role):
        # Iterate through tracked devices to find the controller
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vrsystem.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
                if self.vrsystem.getControllerRoleForTrackedDeviceIndex(i) == role:
                    return i
        self.get_logger().warn('Controller not found; defaulting to index 0')
        return 0

    def timer_callback(self):
        # Get pose
        poses = self.vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            openvr.k_unMaxTrackedDeviceCount)

        pose = poses[self.controller_index]
    
        result, state = self.vrsystem.getControllerState(self.controller_index)
        
        if result:
            # di solito l’asse 1 è il grilletto, ma verifica col tuo hardware
            trigger_value = state.rAxis[2].x
        else:
            self.get_logger().warn(f'${state} qui ${result}')
            trigger_value = 0.0
        
        if pose.bPoseIsValid:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'vr_space'
            # Extract position
            mat = pose.mDeviceToAbsoluteTracking
            msg.pose.position.x = mat[0][3]
            msg.pose.position.y = mat[1][3]
            msg.pose.position.z = mat[2][3]
            # Extract orientation (quaternion)
            # mDeviceToAbsoluteTracking is a 3x4, so need full 3x4->4x4
            # But OpenVR docs give 3x4 matrix, so compute quaternion manually
            rotation_matrix = [
                    [mat[0][0], mat[0][1], mat[0][2]],
                    [mat[1][0], mat[1][1], mat[1][2]],
                    [mat[2][0], mat[2][1], mat[2][2]]
                ]
                
            # Convert to numpy array for scipy
            rotation_matrix = np.array(rotation_matrix, dtype=np.float64)
            r = R.from_matrix(rotation_matrix)
            quat = r.as_quat() 
            qw = quat[3]
            qx = quat[0]
            qy = quat[1]
            qz = quat[2]
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Pose not valid; skipping publish')
        
        trigger_msg = Float32()
        trigger_msg.data = trigger_value
        self.trigger_pub_.publish(trigger_msg)

    def destroy_node(self):
        openvr.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRControllerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
