#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from omni_msgs.msg import OmniButtonEvent
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf_transformations import quaternion_multiply, quaternion_inverse
import math
import numpy as np
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
EEF_FRAME_ID = "tool0"
BASE_FRAME_ID = "base_link"

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

        # self.joint_sub = self.create_subscription(
        #     JointState, '/phantom/joint_states', self.joint_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/phantom/pose', self.pose_callback, 10)
        self.jog_pub = self.create_publisher(
            JointJog, JOINT_TOPIC, 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, TWIST_TOPIC, 10)
        self.grey_button_pub = self.create_subscription(
            OmniButtonEvent, '/phantom/button', self.button_callback, 10)

        
        self.prev_pose = None
        self.prev_positions = None
        self.prev_time = None
        self.max_rate = 1.5  # rad/s
        self.alpha = 0.5 # low pass filter
        self.deadband = 0.01
        self.value = 0.0

        self.filtered_vel = {}
        self.vel_scaling = 10.0

        self.get_logger().info("Phantom → JointJog bridge running.")

    def button_callback(self, msg: OmniButtonEvent):
        self.value = msg.grey_button

    def pose_callback(self, msg: PoseStamped):
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
        vx = dx/dt
        vy = dy/dt
        vz = dz/dt
        
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
            angle-=2*math.pi

        axis = q_rel[:3]
        if np.linalg.norm(axis) > 0:
            axis = axis / np.linalg.norm(axis)
        else:
            axis=[0.0,0.0,0.0]

        wx, wy, wz = (angle*a/dt for a in axis)
        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = EEF_FRAME_ID

        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.x = wx
        twist.twist.angular.y = wy
        twist.twist.angular.z = wz

        
        self.twist_pub.publish(twist)
        self.prev_pose = msg
        self.prev_time = now

    # def joint_callback(self, msg: JointState):
    #     now = self.get_clock().now()
        
    #     if self.prev_positions is None:
    #         self.prev_positions = dict(zip(msg.name, msg.position))
    #         self.prev_time = now
    #         return

    #     dt = (now - self.prev_time).nanoseconds / 1e9
    #     if dt <= 0.0:
    #         return

    #     jog = JointJog()
    #     jog.header.stamp = now.to_msg()
    #     jog.header.frame_id = 'base_link'

    #     for name, pos in zip(msg.name, msg.position):
    #         if name not in self.joint_name_map:
    #             continue  

    #         prev_position = self.prev_positions.get(name, pos)
    #         velocity = (pos - prev_position) / dt
    #         # if name and velocity:
    #         #     print(name, velocity)
            
            

    #         if abs(velocity)<self.deadband:
    #             velocity = 0.0
    #         else:
    #             velocity*= self.vel_scaling

    #         prev_filtered = self.filtered_vel.get(name, 0.0)
    #         filtered_vel = self.alpha*velocity + (1- self.alpha)*prev_filtered
    #         self.filtered_vel[name] = filtered_vel

    #         filtered_vel = max(min(filtered_vel, self.max_rate), -self.max_rate)
    #         jog.joint_names.append(self.joint_name_map[name])
    #         jog.velocities.append(filtered_vel)

    #     if not self.value and jog.joint_names:
    #             self.jog_pub.publish(jog)

    #     self.prev_positions = dict(zip(msg.name, msg.position))
    #     self.prev_time = now

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