#!/usr/bin/env python3
import argparse
import math
import statistics
import struct
import time

import numpy as np
import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def rotate_xyz(point, roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    x, y, z = point

    y, z = y * cr - z * sr, y * sr + z * cr
    x, z = x * cp + z * sp, -x * sp + z * cp
    x, y = x * cy - y * sy, x * sy + y * cy

    return x, y, z


def camera_optical_to_base_link(camera_point, args):
    # RealSense optical frame: x right, y down, z forward.
    # Default mapping assumes the camera looks roughly along base_link +X.
    cam_x, cam_y, cam_z = rotate_xyz(
        camera_point,
        args.camera_roll_deg,
        args.camera_pitch_deg,
        args.camera_yaw_deg,
    )

    base_x = args.camera_x + cam_z
    base_y = args.camera_y - cam_x
    base_z = args.camera_z - cam_y
    return base_x, base_y, base_z


def pack_rgb_from_bgr(bgr):
    b = int(bgr[0])
    g = int(bgr[1])
    r = int(bgr[2])
    return (r << 16) | (g << 8) | b


def in_range(value, lower, upper):
    return lower <= value <= upper


class RealSenseScenePublisher(Node):
    def __init__(self, args):
        super().__init__("realsense_scene_publisher")
        self.args = args
        self.cloud_pub = None
        self.target_pub = None
        self.yolo_model = None
        self.frame_index = 0
        self.last_log_time = 0.0
        self.last_yolo_log_time = 0.0

        if not args.no_obstacles:
            self.cloud_pub = self.create_publisher(PointCloud2, args.obstacle_topic, 5)
        if args.publish_target or args.yolo_model:
            self.target_pub = self.create_publisher(PoseStamped, args.target_topic, 10)
        if args.yolo_model:
            from ultralytics import YOLO

            self.yolo_model = YOLO(args.yolo_model)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(
            rs.stream.depth,
            args.width,
            args.height,
            rs.format.z16,
            args.fps,
        )
        self.config.enable_stream(
            rs.stream.color,
            args.width,
            args.height,
            rs.format.bgr8,
            args.fps,
        )
        self.align = rs.align(rs.stream.color)
        self.pipeline.start(self.config)

        self.get_logger().info(
            f"D435 scene publisher started at {args.width}x{args.height}@{args.fps}"
        )
        if self.cloud_pub is not None:
            self.get_logger().info(
                f"Publishing obstacle PointCloud2 to {args.obstacle_topic} in frame '{args.frame_id}'"
            )
        if args.publish_target:
            self.get_logger().info(
                f"Publishing center target pose to {args.target_topic} in frame '{args.frame_id}'"
            )
        if self.yolo_model is not None:
            target_text = args.target_class if args.target_class else "best detected object"
            self.get_logger().info(
                f"YOLO target mode enabled: model={args.yolo_model}, target={target_text}"
            )

    def close(self):
        self.pipeline.stop()

    def read_center_depth(self, depth_frame, cx, cy):
        half = max(1, self.args.roi_size // 2)
        step = max(1, self.args.roi_step)
        values = []

        for y in range(cy - half, cy + half + 1, step):
            if y < 0 or y >= depth_frame.get_height():
                continue
            for x in range(cx - half, cx + half + 1, step):
                if x < 0 or x >= depth_frame.get_width():
                    continue
                depth = depth_frame.get_distance(x, y)
                if self.args.min_depth <= depth <= self.args.max_depth:
                    values.append(depth)

        if not values:
            return 0.0
        return statistics.median(values)

    def publish_target_pose(self, depth_frame, color_frame, intrinsics):
        if self.target_pub is None or self.yolo_model is not None:
            return

        cx = color_frame.get_width() // 2 + self.args.target_pixel_x_offset
        cy = color_frame.get_height() // 2 + self.args.target_pixel_y_offset
        depth = self.read_center_depth(depth_frame, cx, cy)
        if depth <= 0.0:
            return

        camera_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
        base_point = camera_optical_to_base_link(camera_point, self.args)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = float(base_point[0])
        msg.pose.position.y = float(base_point[1])
        msg.pose.position.z = float(base_point[2] + self.args.target_z_offset)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def publish_yolo_target_pose(self, depth_frame, color_frame, intrinsics):
        if self.target_pub is None or self.yolo_model is None:
            return
        if self.frame_index % max(1, self.args.yolo_every_n) != 0:
            return

        color_image = np.asanyarray(color_frame.get_data())
        results = self.yolo_model(color_image, conf=self.args.yolo_conf, verbose=False)
        best = None

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                name = result.names[cls_id]
                if self.args.target_class and name != self.args.target_class:
                    continue

                conf = float(box.conf[0])
                if best is None or conf > best["conf"]:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    best = {
                        "name": name,
                        "conf": conf,
                        "cx": int((x1 + x2) / 2),
                        "cy": int((y1 + y2) / 2),
                    }

        if best is None:
            return

        cx = max(0, min(depth_frame.get_width() - 1, best["cx"]))
        cy = max(0, min(depth_frame.get_height() - 1, best["cy"]))
        depth = self.read_center_depth(depth_frame, cx, cy)
        if depth <= 0.0:
            return

        camera_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
        base_point = camera_optical_to_base_link(camera_point, self.args)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = float(base_point[0])
        msg.pose.position.y = float(base_point[1])
        msg.pose.position.z = float(base_point[2] + self.args.target_z_offset)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

        now = time.monotonic()
        if now - self.last_yolo_log_time >= 1.0:
            self.last_yolo_log_time = now
            self.get_logger().info(
                f"YOLO target {best['name']} conf={best['conf']:.2f} -> "
                f"base_link=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
            )

    def build_obstacle_points(self, depth_frame, color_frame, intrinsics):
        color_image = np.asanyarray(color_frame.get_data())
        points = []
        step = max(1, self.args.sample_step)

        for y in range(0, depth_frame.get_height(), step):
            for x in range(0, depth_frame.get_width(), step):
                depth = depth_frame.get_distance(x, y)
                if not in_range(depth, self.args.min_depth, self.args.max_depth):
                    continue

                camera_point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                base_x, base_y, base_z = camera_optical_to_base_link(camera_point, self.args)

                if not in_range(base_x, self.args.min_x, self.args.max_x):
                    continue
                if not in_range(base_y, self.args.min_y, self.args.max_y):
                    continue
                if not in_range(base_z, self.args.min_z, self.args.max_z):
                    continue

                rgb = pack_rgb_from_bgr(color_image[y, x])
                points.append((base_x, base_y, base_z, rgb))

        if len(points) <= self.args.max_points:
            return points

        stride = math.ceil(len(points) / self.args.max_points)
        return points[::stride][: self.args.max_points]

    def publish_obstacle_cloud(self, depth_frame, color_frame, intrinsics):
        if self.cloud_pub is None:
            return 0

        points = self.build_obstacle_points(depth_frame, color_frame, intrinsics)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True

        data = bytearray(msg.row_step)
        for index, (x, y, z, rgb) in enumerate(points):
            struct.pack_into("<fffI", data, index * msg.point_step, x, y, z, rgb)
        msg.data = bytes(data)

        self.cloud_pub.publish(msg)
        return len(points)

    def publish_once(self):
        frames = self.pipeline.wait_for_frames(5000)
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return

        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        point_count = self.publish_obstacle_cloud(depth_frame, color_frame, intrinsics)
        self.publish_target_pose(depth_frame, color_frame, intrinsics)
        self.publish_yolo_target_pose(depth_frame, color_frame, intrinsics)
        self.frame_index += 1

        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            if self.cloud_pub is not None:
                self.get_logger().info(
                    f"Published {point_count} obstacle points to {self.args.obstacle_topic}"
                )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Publish D435 obstacle points for Unity visualization."
    )
    parser.add_argument("--obstacle-topic", default="/realsense/obstacle_points")
    parser.add_argument("--target-topic", default="/detected_object_pose")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--width", type=int, default=424)
    parser.add_argument("--height", type=int, default=240)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--publish-hz", type=float, default=5.0)
    parser.add_argument("--sample-step", type=int, default=8)
    parser.add_argument("--max-points", type=int, default=2000)
    parser.add_argument("--min-depth", type=float, default=0.25)
    parser.add_argument("--max-depth", type=float, default=2.00)
    parser.add_argument("--min-x", type=float, default=-1.00)
    parser.add_argument("--max-x", type=float, default=2.00)
    parser.add_argument("--min-y", type=float, default=-1.50)
    parser.add_argument("--max-y", type=float, default=1.50)
    parser.add_argument("--min-z", type=float, default=-0.20)
    parser.add_argument("--max-z", type=float, default=1.50)
    parser.add_argument("--no-obstacles", action="store_true")

    parser.add_argument("--publish-target", action="store_true")
    parser.add_argument("--target-z-offset", type=float, default=0.0)
    parser.add_argument("--target-pixel-x-offset", type=int, default=0)
    parser.add_argument("--target-pixel-y-offset", type=int, default=0)
    parser.add_argument("--roi-size", type=int, default=25)
    parser.add_argument("--roi-step", type=int, default=3)
    parser.add_argument("--yolo-model", help="Optional YOLO model path, for example /mnt/d/Documents/SURF/yolo11n.pt")
    parser.add_argument("--target-class", help="YOLO class to publish, for example bottle")
    parser.add_argument("--yolo-conf", type=float, default=0.40)
    parser.add_argument("--yolo-every-n", type=int, default=1)

    parser.add_argument("--camera-x", type=float, default=0.0)
    parser.add_argument("--camera-y", type=float, default=0.0)
    parser.add_argument("--camera-z", type=float, default=0.0)
    parser.add_argument("--camera-roll-deg", type=float, default=0.0)
    parser.add_argument("--camera-pitch-deg", type=float, default=0.0)
    parser.add_argument("--camera-yaw-deg", type=float, default=0.0)
    return parser


def main():
    args = build_parser().parse_args()
    rclpy.init()
    node = RealSenseScenePublisher(args)
    period = 1.0 / max(0.1, args.publish_hz)

    try:
        while rclpy.ok():
            node.publish_once()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
