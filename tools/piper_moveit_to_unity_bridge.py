#!/usr/bin/env python3
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class MoveItToUnityBridge(Node):
    def __init__(self):
        super().__init__("piper_moveit_to_unity_bridge")
        self.speed = float(self.declare_parameter("speed_percent", 30.0).value)
        self.gripper = 0.0
        self.gripper_effort = 1.0
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.last_positions = [0.0] * 6
        self.has_unity_feedback = False
        self.publisher = self.create_publisher(JointState, "/joint_ctrl_cmd", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.enable_publisher = self.create_publisher(Bool, "/enable_cmd", 10)
        self.feedback_subscription = self.create_subscription(
            JointState,
            "/joint_states_feedback",
            self.forward_unity_feedback,
            10,
        )
        self.arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_arm,
        )
        self.gripper_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            self.execute_gripper,
        )
        self.create_timer(0.02, self.publish_joint_state)
        self.get_logger().info("MoveIt trajectory bridge -> /joint_ctrl_cmd is ready")

    def execute_arm(self, goal_handle):
        self.enable_publisher.publish(Bool(data=True))
        self.get_logger().info(
            f"Received arm trajectory with {len(goal_handle.request.trajectory.points)} points"
        )
        previous_t = 0.0
        for point in goal_handle.request.trajectory.points:
            target_t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            delay = max(0.0, target_t - previous_t)
            previous_t = target_t
            if delay > 0.0:
                time.sleep(delay)
            self.publish_positions(goal_handle.request.trajectory.joint_names, point.positions)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def execute_gripper(self, goal_handle):
        self.enable_publisher.publish(Bool(data=True))
        self.get_logger().info(
            f"Received gripper trajectory with {len(goal_handle.request.trajectory.points)} points"
        )
        for point in goal_handle.request.trajectory.points:
            if point.positions:
                self.gripper = float(point.positions[0])
                self.publish_positions([], [])
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def publish_positions(self, names, positions):
        by_name = {name: float(value) for name, value in zip(names, positions)}
        for index in range(6):
            joint_name = f"joint{index + 1}"
            if joint_name in by_name:
                self.last_positions[index] = by_name[joint_name]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
        msg.position = list(self.last_positions) + [self.gripper]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.speed]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.gripper_effort]
        self.publisher.publish(msg)
        self.get_logger().info(
            "Published /joint_ctrl_cmd positions="
            + ", ".join(f"{value:.3f}" for value in msg.position)
        )

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(self.last_positions)
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        self.joint_state_publisher.publish(msg)

    def forward_unity_feedback(self, msg):
        self.has_unity_feedback = True
        for name, position in zip(msg.name, msg.position):
            if name.startswith("joint"):
                try:
                    index = int(name[5:]) - 1
                except ValueError:
                    continue
                if 0 <= index < len(self.last_positions):
                    self.last_positions[index] = float(position)
                elif index == 6:
                    self.gripper = float(position)
            elif name == "gripper":
                self.gripper = float(position)

        self.publish_joint_state()


def main():
    rclpy.init()
    node = MoveItToUnityBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
