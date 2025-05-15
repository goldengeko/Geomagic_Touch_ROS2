#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog

class PhantomJointToJog(Node):
    def __init__(self):
        super().__init__('phantom_joint_to_jog')

        self.joint_name_map = {
            'waist': 'shoulder_pan_joint',
            'shoulder': 'shoulder_lift_joint',
            'elbow': 'elbow_joint',
            'yaw': 'wrist_1_joint',
            'pitch': 'wrist_2_joint',
            'roll': 'wrist_3_joint',
        }

        self.joint_sub = self.create_subscription(
            JointState, '/phantom/joint_states', self.joint_callback, 10)
        self.jog_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10)

        self.prev_positions = None
        self.prev_time = None
        self.max_rate = 1.5  # rad/s

        self.get_logger().info("Phantom → JointJog bridge running.")

    def joint_callback(self, msg: JointState):
        now = self.get_clock().now()
        if self.prev_positions is None:
            self.prev_positions = dict(zip(msg.name, msg.position))
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        jog = JointJog()
        jog.header.stamp = now.to_msg()
        jog.header.frame_id = 'base_link'

        for name, pos in zip(msg.name, msg.position):
            if name not in self.joint_name_map:
                continue  

            prev_pos = self.prev_positions.get(name, pos)
            velocity = (pos - prev_pos) / dt

            velocity = max(min(velocity, self.max_rate), -self.max_rate)

            jog.joint_names.append(self.joint_name_map[name])
            jog.velocities.append(velocity)

        if jog.joint_names:
            self.jog_pub.publish(jog)

        self.prev_positions = dict(zip(msg.name, msg.position))
        self.prev_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PhantomJointToJog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()