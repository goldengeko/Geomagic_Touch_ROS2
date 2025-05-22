import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from omni_msgs.msg import OmniFeedback

class ForceFeedbackBridge(Node):
    def __init__(self):
        super().__init__('force_feedback_bridge')

        self.force_scale = 1.0   # 1:1 scaling(needs to be adjusted)
        self.alpha = 0.002        # Low-pass filter constant
        self.epsilon = 2.0      # Minimum change to trigger an update (to be adjusted)

        self.filtered_force = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_sent_force = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            10
        )

        self.feedback_pub = self.create_publisher(
            OmniFeedback,
            '/phantom/force_feedback',
            10
        )

        self.get_logger().info("Force feedback bridge with change detection running.")

    def lpf(self, axis, raw_value):
        prev = self.filtered_force.get(axis, 0.0)
        filtered = self.alpha * raw_value + (1 - self.alpha) * prev
        self.filtered_force[axis] = filtered
        return filtered

    def has_significant_change(self, axis):
        return abs(self.filtered_force[axis] - self.prev_sent_force[axis]) > self.epsilon

    def wrench_callback(self, msg: WrenchStamped):
        raw_fx = msg.wrench.force.x * self.force_scale
        raw_fy = msg.wrench.force.y * self.force_scale
        raw_fz = msg.wrench.force.z * self.force_scale

        fx = self.lpf('x', raw_fx)
        fy = self.lpf('y', raw_fy)
        fz = self.lpf('z', raw_fz)

        self.get_logger().debug(f"Raw: ({raw_fx:.2f}, {raw_fy:.2f}, {raw_fz:.2f}) | Filtered: ({fx:.2f}, {fy:.2f}, {fz:.2f})")


        if any(self.has_significant_change(axis) for axis in ['x', 'y', 'z']):
            feedback = OmniFeedback()
            feedback.force.x = fx
            feedback.force.y = fy
            feedback.force.z = fz
            feedback.position.x = 0.0
            feedback.position.y = 0.0
            feedback.position.z = 0.0

            self.feedback_pub.publish(feedback)

            self.prev_sent_force['x'] = -fy
            self.prev_sent_force['y'] = -fz
            self.prev_sent_force['z'] = -fx
        else:
            feedback = OmniFeedback()
            feedback.force.x = 0.0
            feedback.force.y = 0.0
            feedback.force.z = 0.0
            feedback.position.x = 0.0
            feedback.position.y = 0.0
            feedback.position.z = 0.0
            self.feedback_pub.publish(feedback)

def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
