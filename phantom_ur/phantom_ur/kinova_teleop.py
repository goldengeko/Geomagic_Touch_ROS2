#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from omni_msgs.msg import OmniButtonEvent
from geometry_msgs.msg import TwistStamped, PoseStamped
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from tf_transformations import quaternion_multiply, quaternion_inverse
import math
import numpy as np
import sys
import tty
import termios
import signal
import threading
from controller_manager_msgs.srv import ListControllers

# Constants
TWIST_TOPIC = "/twist_controller/commands"
EEF_FRAME_ID = "tool_frame"
BASE_FRAME_ID = "base_link"
    
class PhantomJointToJog(Node):
    def __init__(self):
        super().__init__('phantom_joint_to_jog')

        self.pose_sub = self.create_subscription(
            PoseStamped, '/phantom/pose', self.pose_callback, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, TWIST_TOPIC, 10)
        self.grey_button_pub = self.create_subscription(
            OmniButtonEvent, '/phantom/button', self.button_callback, 10)

        
        self.prev_pose = None
        self.prev_positions = None
        self.prev_time = None
        self.max_rate = 2.0 # rad/s
        self.alpha = 0.005 # low pass filter.reduce for smoother
        self.deadband = 0.01 # rad/s
        self.grey_value = 0.0
        self.white_value = 0.0

        self.filtered_vel = {}
        self.vel_scaling = 2.0  # scaling factor for velocity
        self.twist_scaling = 5.0  # scaling factor for twist commands

        self.mode = 'pose' 

        self.get_logger().info("Phantom → JointJog bridge running in 'pose' mode.")

        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()

    def keyboard_listener(self):
        print(
            """
Reading from keyboard:
---------------------------
Use + or - to increase or decrease velocity scaling.
Use 'p' to switch to 'pose' mode.
Use 'j' to switch to 'joint' mode.
Press Ctrl+C to exit.
"""
        )
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == '+':
                    self.vel_scaling += 2.0
                    self.twist_scaling += 1.0
                    self.get_logger().debug(f"Velocity scaling increased: vel_scaling={self.vel_scaling}, twist_scaling={self.twist_scaling}")
                elif key == '-':
                    self.vel_scaling = max(1.0, self.vel_scaling - 1.0)
                    self.twist_scaling = max(0.5, self.twist_scaling - 0.5)
                    self.get_logger().debug(f"Velocity scaling decreased: vel_scaling={self.vel_scaling}, twist_scaling={self.twist_scaling}")
                elif key.lower() == 'p':
                    self.mode = 'pose'
                    self.get_logger().info("Switched to 'pose' mode.")
                elif key.lower() == 'j':
                    self.mode = 'joint'
                    self.get_logger().info("Switched to 'joint' mode.")
                elif key == '\x03':  
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def button_callback(self, msg: OmniButtonEvent):
        self.grey_value = msg.grey_button
        self.white_value = msg.white_button

    def pose_callback(self, msg: PoseStamped):
        if self.mode != 'pose':
            return  

        now = self.get_clock().now()

        if self.prev_pose is None:
            self.prev_pose = msg
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dx = msg.pose.position.x - self.prev_pose.pose.position.x
        dy = msg.pose.position.y - self.prev_pose.pose.position.y
        dz = msg.pose.position.z - self.prev_pose.pose.position.z
        vx = dx / dt
        vy = dy / dt
        vz = dz / dt

        q1 = [
            self.prev_pose.pose.orientation.x,
            self.prev_pose.pose.orientation.y,
            self.prev_pose.pose.orientation.z,
            self.prev_pose.pose.orientation.w
        ]
        q2 = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        q_rel = quaternion_multiply(q2, quaternion_inverse(q1))
        angle = 2 * math.acos(max(-1.0, min(1.0, q_rel[3])))
        if angle > math.pi:
            angle -= 2 * math.pi

        axis = q_rel[:3]
        axis_norm = np.linalg.norm(axis)
        axis = axis / axis_norm if axis_norm > 0 else [0.0, 0.0, 0.0]
        wx, wy, wz = (angle * a / dt for a in axis)

        def lpf(key, raw_value):
            prev = self.filtered_vel.get(key, 0.0)
            filtered = self.alpha * raw_value + (1 - self.alpha) * prev
            self.filtered_vel[key] = filtered
            return filtered

        vx = lpf('vx', vx)
        vy = lpf('vy', vy)
        vz = lpf('vz', vz)
        wx = lpf('wx', wx)
        wy = lpf('wy', wy)
        wz = lpf('wz', wz)

        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = BASE_FRAME_ID if self.white_value else EEF_FRAME_ID

        twist.twist.linear.x = vx * self.twist_scaling
        twist.twist.linear.y = vy * self.twist_scaling
        twist.twist.linear.z = vz * self.twist_scaling
        twist.twist.angular.x = wx * self.twist_scaling
        twist.twist.angular.y = wy * self.twist_scaling
        twist.twist.angular.z = wz * self.twist_scaling

        if not self.grey_value and twist.twist.linear.x != 0.0:
            self.twist_pub.publish(twist)

        self.prev_pose = msg
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