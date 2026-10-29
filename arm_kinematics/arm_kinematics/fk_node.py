"""Nodo ROS 2: /joint_states -> cinemática directa -> /end_effector_pose."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from arm_kinematics.kinematics import end_effector_pose


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')
        # Parámetros: se pueden cambiar sin tocar el código
        self.declare_parameter('link_lengths', [0.30, 0.25, 0.10])
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('base_frame', 'base_link')

        self.pub = self.create_publisher(PoseStamped, 'end_effector_pose', 10)
        self.sub = self.create_subscription(
            JointState, 'joint_states', self.on_joint_states, 10)
        self.get_logger().info('fk_node listo: escuchando /joint_states')

    def on_joint_states(self, msg: JointState):
        names = list(self.get_parameter('joint_names').value)
        lengths = list(self.get_parameter('link_lengths').value)

        # Ordena los ángulos según joint_names (el mensaje puede traerlos en otro orden)
        try:
            q = [msg.position[msg.name.index(n)] for n in names]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f'/joint_states no contiene {names}', throttle_duration_sec=5.0)
            return

        x, y, phi = end_effector_pose(q, lengths)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('base_frame').value
        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.position.z = 0.0
        # Rotación phi alrededor de z expresada como cuaternión
        out.pose.orientation.z = math.sin(phi / 2.0)
        out.pose.orientation.w = math.cos(phi / 2.0)
        self.pub.publish(out)

        self.get_logger().info(
            f'q = {[round(math.degrees(v), 1) for v in q]} deg -> '
            f'x = {x:.4f} m, y = {y:.4f} m, phi = {math.degrees(phi):.1f} deg',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
