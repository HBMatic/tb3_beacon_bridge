import math
import time

import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePositionAngle
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


class BeaconBridgeNode(Node):

    def __init__(self):
        super().__init__("beacon_bridge_node")

        self.declare_parameter("robot_namespace", "tb3_1")
        self.declare_parameter("beacon_a_addr",   1)
        self.declare_parameter("beacon_b_addr",   2)
        self.declare_parameter("beacon_timeout",  1.0)
        self.declare_parameter("pos_sigma",       0.30)
        self.declare_parameter("heading_sigma",   0.2236)

        ns           = self.get_parameter("robot_namespace").value
        self.addr_a  = self.get_parameter("beacon_a_addr").value
        self.addr_b  = self.get_parameter("beacon_b_addr").value
        self.timeout     = self.get_parameter("beacon_timeout").value
        self.pos_var     = self.get_parameter("pos_sigma").value ** 2
        self.heading_var = self.get_parameter("heading_sigma").value ** 2

        self.last_a = None  # (x, y, timestamp)
        self.last_b = None  # (x, y, angle_deg, timestamp)

        # Cross-subscribe to both driver topics so port assignment does not matter.
        # Each callback filters by hardware address from the message.
        for topic_addr in [self.addr_a, self.addr_b]:
            self.create_subscription(
                HedgePositionAngle,
                f"/beacon_{topic_addr}/hedgehog_pos_ang",
                self._cb_dispatch, 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, f"/{ns}/marvelmind_pose", 10)
        self.mode_pub = self.create_publisher(
            String, f"/{ns}/beacon_mode", 10)

        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f"BeaconBridge ready — ns={ns} addr_a={self.addr_a} addr_b={self.addr_b} "
            f"(heading from beacon_{self.addr_b}, port-agnostic)")

    def _cb_dispatch(self, msg):
        if msg.address == self.addr_a:
            self.last_a = (msg.x_m, msg.y_m, time.time())
        elif msg.address == self.addr_b:
            self.last_b = (msg.x_m, msg.y_m, msg.angle, time.time())

    def _fresh(self, data):
        return data is not None and (time.time() - data[-1]) < self.timeout

    def _publish(self):
        a_ok = self._fresh(self.last_a)
        b_ok = self._fresh(self.last_b)

        if not a_ok and not b_ok:
            return

        pose = PoseWithCovarianceStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        if a_ok and b_ok:
            xa, ya, _           = self.last_a
            xb, yb, ang_deg, _  = self.last_b
            x   = (xa + xb) / 2.0
            y   = (ya + yb) / 2.0
            yaw = math.radians(ang_deg)
            pos_var = self.pos_var / 2.0
            yaw_var = self.heading_var
            mode = "paired"
        elif a_ok:
            x, y, _ = self.last_a
            yaw, pos_var, yaw_var, mode = 0.0, self.pos_var, 1e6, f"single_{self.addr_a}"
        else:
            x, y, ang_deg, _ = self.last_b
            yaw, pos_var, yaw_var, mode = 0.0, self.pos_var, 1e6, f"single_{self.addr_b}"

        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        cov[0]  = pos_var
        cov[7]  = pos_var
        cov[14] = 1e6
        cov[21] = 1e6
        cov[28] = 1e6
        cov[35] = yaw_var
        pose.pose.covariance = cov

        self.pose_pub.publish(pose)
        m = String(); m.data = mode
        self.mode_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BeaconBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
