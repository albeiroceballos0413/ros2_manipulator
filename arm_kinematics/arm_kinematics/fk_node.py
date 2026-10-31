"""Nodo ROS 2: cinemática directa por topic (/joint_states) y por service (/compute_fk)."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from arm_interfaces.srv import ComputeFK
from arm_kinematics.kinematics import end_effector_pose


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node')
        self.declare_parameter('link_lengths', [0.30, 0.25, 0.10])
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3'])
        self.declare_parameter('base_frame', 'base_link')

        self.pub = self.create_publisher(PoseStamped, 'end_effector_pose', 10)
        self.sub = self.create_subscription(
            JointState, 'joint_states', self.on_joint_states, 10)
        self.srv = self.create_service(ComputeFK, 'compute_fk', self.on_compute_fk)
        self.get_logger().info('fk_node listo: topic /joint_states y service /compute_fk')

    def lengths(self):
        return list(self.get_parameter('link_lengths').value)

    # ---------- Topic ----------
    def on_joint_states(self, msg: JointState):
        names = list(self.get_parameter('joint_names').value)
        try:
            q = [msg.position[msg.name.index(n)] for n in names]
        except (ValueError, IndexError):
            self.get_logger().warn(
                f'/joint_states no contiene {names}', throttle_duration_sec=5.0)
            return

        x, y, phi = end_effector_pose(q, self.lengths())

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('base_frame').value
        out.pose.position.x = float(x)
        out.pose.position.y = float(y)
        out.pose.orientation.z = math.sin(phi / 2.0)
        out.pose.orientation.w = math.cos(phi / 2.0)
        self.pub.publish(out)

        self.get_logger().info(
            f'q = {[round(math.degrees(v), 1) for v in q]} deg -> '
            f'x = {x:.4f} m, y = {y:.4f} m, phi = {math.degrees(phi):.1f} deg',
            throttle_duration_sec=1.0)

    # ---------- Service ----------
    def on_compute_fk(self, request, response):
        lengths = self.lengths()
        q = list(request.q)
        if len(q) != len(lengths):
            response.success = False
            response.message = f'Se esperaban {len(lengths)} ángulos, llegaron {len(q)}'
            return response

        x, y, phi = end_effector_pose(q, lengths)
        response.x, response.y, response.phi = float(x), float(y), float(phi)
        response.success = True
        response.message = 'ok'
        self.get_logger().info(f'compute_fk({q}) -> ({x:.4f}, {y:.4f}, {phi:.4f})')
        return response


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
