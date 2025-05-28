import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped, TransformStamped, Vector3, PoseStamped, Vector3Stamped
from omni_msgs.msg import OmniFeedback, OmniButtonEvent
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.parameter import Parameter
from tf2_geometry_msgs import do_transform_vector3
from collections import deque
import numpy as np

class ForceFeedbackBridge(Node):
    def __init__(self):
        super().__init__('force_feedback_bridge')

        self.declare_parameter('force_scale', 20.0)
        self.force_scale = self.get_parameter('force_scale').get_parameter_value().double_value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            qos
        )

        self.position_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.position_callback,
            qos
        )

        self.button_sub = self.create_subscription(
            OmniButtonEvent,
            '/phantom/button',
            self.button_callback,
            qos
        )

        self.feedback_pub = self.create_publisher(
            OmniFeedback,
            '/phantom/force_feedback',
            qos
        )

        self.latest_position = Vector3(x=0.0, y=0.0, z=0.0)
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.max_force = 2.0
        self.previous_force = Vector3(x=0.0, y=0.0, z=0.0)
        self.alpha = 0.005

        self.white = 0.0

        self.force_buffer_size = 100
        self.force_buffers = {
            'x': deque(maxlen=self.force_buffer_size),
            'y': deque(maxlen=self.force_buffer_size),
            'z': deque(maxlen=self.force_buffer_size)
        }

        self.previous_mean_x = 0.0
        self.previous_mean_y = 0.0
        self.previous_mean_z = 0.0

        self.get_logger().info("Force feedback bridge running.")

    def button_callback(self, msg: OmniButtonEvent):
        self.white = msg.white_button

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'force_scale' and param.type_ == Parameter.Type.DOUBLE:
                if param.value > 0.0:
                    self.force_scale = param.value
                    self.get_logger().info(f"Updated force_scale to {self.force_scale}")
                else:
                    self.get_logger().warn("force_scale must be positive")
                    return rclpy.parameter.Parameter.SetParametersResult(successful=False)
        return rclpy.parameter.Parameter.SetParametersResult(successful=True)

    def position_callback(self, msg):
        self.latest_position.x = msg.pose.position.x
        self.latest_position.y = msg.pose.position.y
        self.latest_position.z = msg.pose.position.z

    def wrench_callback(self, msg: WrenchStamped):
        try:
            target_frame = 'base'
            source_frame = 'wrist_3_link'
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            force = msg.wrench.force
            transformed_force = self.transform_vector(transform, force)

            filtered_force = Vector3()
            filtered_force.x = self.alpha * transformed_force.x + (1 - self.alpha) * self.previous_force.x
            filtered_force.y = self.alpha * transformed_force.y + (1 - self.alpha) * self.previous_force.y
            filtered_force.z = self.alpha * transformed_force.z + (1 - self.alpha) * self.previous_force.z
            self.previous_force = filtered_force

            self.force_buffers['x'].append(filtered_force.x)
            self.force_buffers['y'].append(filtered_force.y)
            self.force_buffers['z'].append(filtered_force.z)

            fx = apply_median_filter(self.force_buffers['x'])
            fy = apply_median_filter(self.force_buffers['y'])
            fz = apply_median_filter(self.force_buffers['z'])

            fx = sum(self.force_buffers['x']) / len(self.force_buffers['x'])
            fy = sum(self.force_buffers['y']) / len(self.force_buffers['y'])
            fz = sum(self.force_buffers['z']) / len(self.force_buffers['z'])

            diff_x = abs(fx - self.previous_mean_x)
            diff_y = abs(fy - self.previous_mean_y)
            diff_z = abs(fz - self.previous_mean_z)

            diff_std = np.std([diff_x, diff_y, diff_z])
            threshold = 15 #max(0.1, diff_std * 3) 
            self.get_logger().info(f"Threshold: {threshold}, Rolling mean differences: dx={diff_x}, dy={diff_y}, dz={diff_z}")

            if diff_x < threshold and diff_y < threshold and diff_z < threshold or self.white:
                self.get_logger().info(f"No significant change detected. Resetting forces to zero.")
                fx, fy, fz = 0.0, 0.0, 0.0
            else:
                self.get_logger().info(f"Significant change detected. Forces retained.")

            self.previous_mean_x = fx
            self.previous_mean_y = fy
            self.previous_mean_z = fz

            fx *= self.force_scale
            fy *= self.force_scale
            fz *= self.force_scale

            fx = max(min(fx, self.max_force), -self.max_force)
            fy = max(min(fy, self.max_force), -self.max_force)
            fz = max(min(fz, self.max_force), -self.max_force)

            feedback = OmniFeedback()
            feedback.force.x = fx
            feedback.force.y = fy
            feedback.force.z = fz
            feedback.position = self.latest_position

            self.feedback_pub.publish(feedback)

        except TransformException as e:
            self.get_logger().warn(f"Transform failed: {str(e)}. Using raw force data.")
            fx = msg.wrench.force.x * self.force_scale
            fy = msg.wrench.force.y * self.force_scale
            fz = msg.wrench.force.z * self.force_scale

            fx = max(min(fx, self.max_force), -self.max_force)
            fy = max(min(fy, self.max_force), -self.max_force)
            fz = max(min(fz, self.max_force), -self.max_force)

            feedback = OmniFeedback()
            feedback.force.x = fx
            feedback.force.y = fy
            feedback.force.z = fz
            feedback.position = self.latest_position
            self.feedback_pub.publish(feedback)

    def transform_vector(self, transform: TransformStamped, vector: Vector3):
        vector_stamped = Vector3Stamped()
        vector_stamped.vector = vector
        vector_stamped.header.frame_id = transform.header.frame_id
        vector_stamped.header.stamp = rclpy.time.Time().to_msg()

        transformed_vector_stamped = do_transform_vector3(vector_stamped, transform)
        return transformed_vector_stamped.vector

def apply_median_filter(data):
    return np.median(data)

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