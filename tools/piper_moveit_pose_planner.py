#!/usr/bin/env python3
"""Plan and execute a PiPER arm pose target using MoveIt 2 + RRT.

This node receives a target end-effector pose (position + orientation) either from
command-line arguments or from a geometry_msgs/PoseStamped topic, builds a
MoveIt ``MotionPlanRequest`` with an RRT-based planner, and sends it to the
``/move_action`` action server exposed by MoveIt's ``move_group`` node.  When the
project is launched with ``piper_moveit_to_unity_bridge.py`` running, MoveIt's
trajectory execution sends the resulting ``FollowJointTrajectory`` action to
``/arm_controller/follow_joint_trajectory``, which the bridge forwards to Unity as
``/joint_ctrl_cmd``.

Examples
--------
Move the end-effector (link6) to a Cartesian position with default orientation::

    ros2 run piper_tools piper_moveit_pose_planner.py -- \
        --x 0.3 --y 0.0 --z 0.4

Specify orientation as roll/pitch/yaw (degrees)::

    ros2 run piper_tools piper_moveit_pose_planner.py -- \
        --x 0.3 --y 0.0 --z 0.4 --roll 0 --pitch 45 --yaw 0

Use RRTConnect instead of the default RRT::

    ros2 run piper_tools piper_moveit_pose_planner.py -- \
        --x 0.3 --y 0.0 --z 0.4 --planner RRTConnect

Plan near an object pose published by RealSense/Unity, stopping 10 cm before the
object in the base_link XY plane and 5 cm above its reported point::

    python3 tools/piper_moveit_pose_planner.py \
        --pose-topic /detected_object_pose \
        --position-only \
        --approach-standoff 0.10 \
        --z-offset 0.05 \
        --planner RRTConnect
"""

import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


def _quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Convert intrinsic Tait-Bryan angles (roll, pitch, yaw) to a quaternion (x, y, z, w).

    Re-implementation of ``tf_transformations.quaternion_from_euler`` to avoid an
    extra runtime dependency in ROS2 environments where the package may not be
    available.
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


DEFAULT_PLANNING_GROUP = "arm"
DEFAULT_END_EFFECTOR_LINK = "link6"
DEFAULT_PLANNER = ""
DEFAULT_PIPELINE = "ompl"
DEFAULT_PLANNING_TIME = 5.0


class PosePlannerNode(Node):
    """ROS2 node that plans and executes pose targets for the PiPER arm."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("piper_moveit_pose_planner")
        self.args = args
        self.planning_group = args.planning_group
        self.end_effector_link = args.end_effector_link
        self._latest_pose: Optional[PoseStamped] = None

        self.get_logger().info(
            f"Initializing MoveGroup action client for group '{self.planning_group}'"
        )
        self.move_action_client = ActionClient(self, MoveGroup, "/move_action")
        server_ready = self.move_action_client.wait_for_server(timeout_sec=10.0)
        if not server_ready:
            raise RuntimeError(
                "/move_action action server is not available. "
                "Please start the MoveIt move_group node first. "
                "From the menu choose option 3 (MoveIt RViz) or option 14 (MoveIt move_group headless). "
                "Or run: ros2 launch piper_no_gripper_moveit move_group.launch.py"
            )
        self.get_logger().info("Connected to /move_action")

        if args.pose_topic:
            self.pose_subscription = self.create_subscription(
                PoseStamped,
                args.pose_topic,
                self._on_pose,
                1,
            )
            self.get_logger().info(f"Waiting for pose on topic '{args.pose_topic}'...")

    def _on_pose(self, msg: PoseStamped) -> None:
        """Handle a pose message from the subscribed topic."""
        if self._latest_pose is not None:
            return
        self.get_logger().info(
            f"Received pose target: frame={msg.header.frame_id}, "
            f"position=({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )
        self._latest_pose = msg

    def _apply_target_offsets(self, pose: PoseStamped) -> PoseStamped:
        """Offset an object pose into a safer end-effector approach target."""
        standoff = max(0.0, self.args.approach_standoff)
        z_offset = self.args.z_offset

        if standoff <= 0.0 and abs(z_offset) <= 1e-9:
            return pose

        adjusted = PoseStamped()
        adjusted.header = pose.header
        adjusted.pose = Pose()
        adjusted.pose.position.x = pose.pose.position.x
        adjusted.pose.position.y = pose.pose.position.y
        adjusted.pose.position.z = pose.pose.position.z + z_offset
        adjusted.pose.orientation = pose.pose.orientation

        if standoff > 0.0:
            xy_norm = math.hypot(pose.pose.position.x, pose.pose.position.y)
            if xy_norm > 1e-6:
                adjusted.pose.position.x -= standoff * pose.pose.position.x / xy_norm
                adjusted.pose.position.y -= standoff * pose.pose.position.y / xy_norm
            else:
                adjusted.pose.position.x -= standoff

        self.get_logger().info(
            f"Adjusted object pose to approach target: "
            f"position=({adjusted.pose.position.x:.3f}, "
            f"{adjusted.pose.position.y:.3f}, {adjusted.pose.position.z:.3f}), "
            f"standoff={standoff:.3f} m, z_offset={z_offset:.3f} m"
        )
        return adjusted

    def _build_pose(self) -> PoseStamped:
        """Build a PoseStamped from CLI arguments or the subscribed topic."""
        if self.args.pose_topic:
            deadline = time.monotonic() + self.args.pose_timeout
            while self._latest_pose is None and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)

            pose = self._latest_pose
            if pose is None:
                raise RuntimeError(
                    f"No pose received on topic '{self.args.pose_topic}' "
                    f"within {self.args.pose_timeout:.1f} seconds"
                )
            return self._apply_target_offsets(pose)

        pose = PoseStamped()
        pose.header.frame_id = self.args.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.args.x
        pose.pose.position.y = self.args.y
        pose.pose.position.z = self.args.z

        if self.args.use_quaternion:
            pose.pose.orientation.x = self.args.qx
            pose.pose.orientation.y = self.args.qy
            pose.pose.orientation.z = self.args.qz
            pose.pose.orientation.w = self.args.qw
        else:
            roll = math.radians(self.args.roll)
            pitch = math.radians(self.args.pitch)
            yaw = math.radians(self.args.yaw)
            qx, qy, qz, qw = _quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

        return pose

    def _build_goal_constraints(self, target_pose: PoseStamped) -> Constraints:
        """Build position + orientation constraints for the end-effector pose."""
        constraints = Constraints()

        # Spherical position constraint around the target position.
        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance]
        position_constraint.constraint_region.primitives.append(sphere)

        region_pose = Pose()
        region_pose.position = target_pose.pose.position
        region_pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses.append(region_pose)

        constraints.position_constraints.append(position_constraint)

        if not self.args.position_only:
            # Orientation constraint around the target orientation.
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = target_pose.header
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.args.orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = self.args.orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = self.args.orientation_tolerance
            orientation_constraint.weight = 1.0

            constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def plan_and_execute(self) -> bool:
        """Plan a trajectory to the target pose and execute it."""
        target_pose = self._build_pose()

        orientation_info = "position-only" if self.args.position_only else "position+orientation"
        self.get_logger().info(
            f"Planning to: frame={target_pose.header.frame_id}, "
            f"position=({target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, "
            f"{target_pose.pose.position.z:.3f}), "
            f"planner={self.args.planner or 'default'}, pipeline={self.args.pipeline}, "
            f"mode={orientation_info}"
        )

        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.pipeline_id = self.args.pipeline
        if self.args.planner:
            request.planner_id = self.args.planner
        request.allowed_planning_time = self.args.planning_time
        request.num_planning_attempts = self.args.attempts
        request.max_velocity_scaling_factor = self.args.velocity_scaling
        request.max_acceleration_scaling_factor = self.args.acceleration_scaling
        request.goal_constraints = [self._build_goal_constraints(target_pose)]

        # Tell MoveGroup to use the current planning scene and current robot state.
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True
        planning_scene_diff.robot_state = RobotState()
        planning_scene_diff.robot_state.is_diff = True

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.look_around = False
        planning_options.replan = False
        planning_options.planning_scene_diff = planning_scene_diff

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        self.get_logger().info("Sending MoveGroup goal...")
        send_future = self.move_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("Failed to send MoveGroup goal")
            return False
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return False

        self.get_logger().info("Goal accepted, waiting for plan + execution...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.args.execution_timeout
        )
        if not result_future.done():
            self.get_logger().error("Timeout waiting for MoveGroup result")
            return False

        result = result_future.result().result
        success = result.error_code.val == MoveItErrorCodes.SUCCESS
        if success:
            self.get_logger().info("MoveGroup plan + execution succeeded")
        else:
            self.get_logger().error(
                f"MoveGroup plan + execution failed with error_code={result.error_code.val}"
            )
        return success

    def destroy_node(self) -> None:
        """Clean up action client resources."""
        self.move_action_client.destroy()
        super().destroy_node()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser."""
    parser = argparse.ArgumentParser(
        description="Plan and execute a PiPER arm pose target using MoveIt 2 + RRT."
    )

    pose = parser.add_argument_group("target pose")
    pose.add_argument("--x", type=float, default=0.3, help="Target X coordinate (m)")
    pose.add_argument("--y", type=float, default=0.0, help="Target Y coordinate (m)")
    pose.add_argument("--z", type=float, default=0.4, help="Target Z coordinate (m)")
    pose.add_argument(
        "--roll", type=float, default=0.0, help="Target roll (degrees)"
    )
    pose.add_argument(
        "--pitch", type=float, default=0.0, help="Target pitch (degrees)"
    )
    pose.add_argument(
        "--yaw", type=float, default=0.0, help="Target yaw (degrees)"
    )
    pose.add_argument("--qx", type=float, default=0.0, help="Quaternion x")
    pose.add_argument("--qy", type=float, default=0.0, help="Quaternion y")
    pose.add_argument("--qz", type=float, default=0.0, help="Quaternion z")
    pose.add_argument("--qw", type=float, default=1.0, help="Quaternion w")
    pose.add_argument(
        "--use-quaternion",
        action="store_true",
        help="Use --qx/--qy/--qz/--qw instead of roll/pitch/yaw",
    )
    pose.add_argument(
        "--pose-topic",
        type=str,
        default="",
        help="Subscribe to a geometry_msgs/PoseStamped topic instead of using CLI pose",
    )
    pose.add_argument(
        "--pose-timeout",
        type=float,
        default=10.0,
        help="Timeout in seconds while waiting for --pose-topic (default: 10.0)",
    )
    pose.add_argument(
        "--frame-id",
        type=str,
        default="base_link",
        help="Reference frame for the target pose (default: base_link)",
    )
    pose.add_argument(
        "--position-tolerance",
        type=float,
        default=0.001,
        help="Goal position tolerance (m, default: 0.001)",
    )
    pose.add_argument(
        "--orientation-tolerance",
        type=float,
        default=0.1,
        help="Goal orientation tolerance (rad, default: 0.1)",
    )
    pose.add_argument(
        "--position-only",
        action="store_true",
        help="Only constrain position; let MoveIt choose a feasible orientation",
    )
    pose.add_argument(
        "--approach-standoff",
        type=float,
        default=0.0,
        help=(
            "Stop this many meters before a topic pose in the base_link XY plane. "
            "Useful when the topic pose is an object center instead of an end-effector target."
        ),
    )
    pose.add_argument(
        "--z-offset",
        type=float,
        default=0.0,
        help="Add this many meters to the target Z coordinate before planning.",
    )

    planning = parser.add_argument_group("planning configuration")
    planning.add_argument(
        "--planning-group",
        type=str,
        default=DEFAULT_PLANNING_GROUP,
        help=f"MoveIt planning group (default: {DEFAULT_PLANNING_GROUP})",
    )
    planning.add_argument(
        "--end-effector-link",
        type=str,
        default=DEFAULT_END_EFFECTOR_LINK,
        help=f"End-effector link name (default: {DEFAULT_END_EFFECTOR_LINK})",
    )
    planning.add_argument(
        "--planner",
        type=str,
        default=DEFAULT_PLANNER,
        help=f"OMPL planner id (default: {DEFAULT_PLANNER})",
    )
    planning.add_argument(
        "--pipeline",
        type=str,
        default=DEFAULT_PIPELINE,
        help=f"MoveIt planning pipeline id (default: {DEFAULT_PIPELINE})",
    )
    planning.add_argument(
        "--planning-time",
        type=float,
        default=DEFAULT_PLANNING_TIME,
        help=f"Maximum planning time in seconds (default: {DEFAULT_PLANNING_TIME})",
    )
    planning.add_argument(
        "--attempts",
        type=int,
        default=1,
        help="Number of planning attempts (default: 1)",
    )
    planning.add_argument(
        "--velocity-scaling",
        type=float,
        default=0.3,
        help="Max velocity scaling factor (default: 0.3)",
    )
    planning.add_argument(
        "--acceleration-scaling",
        type=float,
        default=0.3,
        help="Max acceleration scaling factor (default: 0.3)",
    )
    planning.add_argument(
        "--execution-timeout",
        type=float,
        default=60.0,
        help="Timeout in seconds for action result (default: 60.0)",
    )

    return parser


def main() -> int:
    """Entry point."""
    parser = build_parser()
    args = parser.parse_args()

    if not args.pose_topic:
        if args.use_quaternion:
            norm = math.sqrt(args.qx**2 + args.qy**2 + args.qz**2 + args.qw**2)
            if abs(norm - 1.0) > 1e-3:
                print(
                    f"Warning: quaternion norm is {norm:.4f}, normalizing.",
                    file=sys.stderr,
                )
                args.qx /= norm
                args.qy /= norm
                args.qz /= norm
                args.qw /= norm

    rclpy.init()
    node: Optional[PosePlannerNode] = None
    try:
        node = PosePlannerNode(args)
        success = node.plan_and_execute()
        return 0 if success else 1
    except Exception as exc:  # pragma: no cover - runtime environment errors
        if node is not None:
            node.get_logger().error(f"Pose planner failed: {exc}")
        else:
            print(f"Pose planner failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
