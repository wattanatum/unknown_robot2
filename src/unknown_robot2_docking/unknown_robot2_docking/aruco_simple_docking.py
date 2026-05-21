#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped

from tf2_ros import Buffer, TransformListener


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class ArucoSimpleDocking(Node):
    def __init__(self):
        super().__init__("aruco_simple_docking")

        # ==============================
        # Camera / robot topics
        # ==============================
        self.declare_parameter("image_topic", "/camera/depth/image")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # ==============================
        # ArUco settings
        # ==============================
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.125)

        # ==============================
        # Final docking settings
        # ==============================
        self.declare_parameter("target_distance", 0.25)
        self.declare_parameter("max_linear_speed", 0.02)
        self.declare_parameter("max_angular_speed", 0.10)
        self.declare_parameter("linear_kp", 0.25)
        self.declare_parameter("angular_kp", 0.8)
        self.declare_parameter("center_tolerance_px", 30.0)

        # ==============================
        # Enable robot movement
        # ==============================
        self.declare_parameter("enable_docking", False)

        # ==============================
        # TF check settings
        # Nav2 should already align these.
        # This node only checks them.
        # ==============================
        self.declare_parameter("use_tf_alignment", True)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("dock_frame", "aruco_docking_marker")

        # Robot Y must be within this range of ArUco Y.
        self.declare_parameter("y_threshold", 0.05)

        # Robot yaw must face opposite of ArUco yaw.
        # desired_robot_yaw = aruco_yaw + 180 deg
        self.declare_parameter("yaw_threshold_deg", 5.0)

        # ==============================
        # Read parameters
        # ==============================
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)

        self.target_distance = float(self.get_parameter("target_distance").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.linear_kp = float(self.get_parameter("linear_kp").value)
        self.angular_kp = float(self.get_parameter("angular_kp").value)
        self.center_tolerance_px = float(self.get_parameter("center_tolerance_px").value)

        self.use_tf_alignment = bool(self.get_parameter("use_tf_alignment").value)
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.dock_frame = self.get_parameter("dock_frame").value

        self.y_threshold = float(self.get_parameter("y_threshold").value)
        self.yaw_threshold_deg = float(self.get_parameter("yaw_threshold_deg").value)
        self.yaw_threshold_rad = math.radians(self.yaw_threshold_deg)

        # ==============================
        # Camera state
        # ==============================
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.image_width = 640
        self.image_height = 480

        # ==============================
        # ArUco state
        # ==============================
        self.last_seen_time = None
        self.current_distance = None
        self.current_pixel_error_x = None
        self.current_marker_center_x = None
        self.current_marker_center_y = None

        # ==============================
        # TF state
        # ==============================
        self.tf_ok = False
        self.y_ok = False
        self.yaw_ok = False

        self.current_robot_y = None
        self.current_dock_y = None
        self.current_y_error = None
        self.current_y_diff = None

        self.current_robot_yaw = None
        self.current_dock_yaw = None
        self.current_desired_robot_yaw = None
        self.current_yaw_error_rad = None
        self.current_yaw_diff_rad = None

        # ==============================
        # Docking state
        # ==============================
        self.docked = False

        # ==============================
        # OpenCV ArUco
        # Use older API because newer ArucoDetector can segfault
        # on some OpenCV versions.
        # ==============================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # ==============================
        # TF
        # ==============================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==============================
        # ROS interfaces
        # ==============================
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco_dock_pose_camera",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/aruco_docking/debug_image",
            10,
        )

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Aruco final docking node started")
        self.get_logger().info("Nav2 should align robot to staging pose first.")
        self.get_logger().info("This node only checks TF alignment, then performs final ArUco docking.")
        self.get_logger().info("Yaw rule: robot yaw must be ArUco yaw + 180 deg.")
        self.get_logger().info(f"image_topic: {self.image_topic}")
        self.get_logger().info(f"camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info(f"cmd_vel_topic: {self.cmd_vel_topic}")
        self.get_logger().info(f"marker_id: {self.marker_id}")
        self.get_logger().info(f"marker_size: {self.marker_size}")
        self.get_logger().info(f"use_tf_alignment: {self.use_tf_alignment}")
        self.get_logger().info(f"global_frame: {self.global_frame}")
        self.get_logger().info(f"robot_frame: {self.robot_frame}")
        self.get_logger().info(f"dock_frame: {self.dock_frame}")
        self.get_logger().info(f"y_threshold: {self.y_threshold}")
        self.get_logger().info(f"yaw_threshold_deg: {self.yaw_threshold_deg}")

    # ============================================================
    # CameraInfo callback
    # ============================================================
    def camera_info_callback(self, msg):
        if self.fx is not None:
            return

        self.image_width = msg.width
        self.image_height = msg.height

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

        self.get_logger().info("Camera info received")
        self.get_logger().info(
            f"image size: {self.image_width} x {self.image_height}"
        )
        self.get_logger().info(
            f"fx={self.fx:.2f}, fy={self.fy:.2f}, "
            f"cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

    # ============================================================
    # ROS Image to OpenCV, no cv_bridge
    # ============================================================
    def ros_image_to_cv2(self, msg):
        if msg.encoding not in ["rgb8", "bgr8"]:
            raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

        img = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = msg.height * msg.width * 3

        if img.size != expected_size:
            raise RuntimeError(
                f"Image size mismatch. Got {img.size}, expected {expected_size}"
            )

        img = img.reshape((msg.height, msg.width, 3))

        if msg.encoding == "rgb8":
            frame_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = img.copy()

        return frame_bgr

    def cv2_to_ros_rgb_image(self, frame_bgr, header):
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header = header
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame_rgb.tobytes()

        return msg

    # ============================================================
    # Image callback: detect ArUco marker and publish debug image
    # ============================================================
    def image_callback(self, msg):
        if self.fx is None:
            self.get_logger().warn(
                "Waiting for camera_info...",
                throttle_duration_sec=2.0,
            )
            return

        try:
            frame = self.ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )
        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {e}")
            return

        detected = False

        if ids is not None:
            ids_flat = ids.flatten()

            for i, detected_id in enumerate(ids_flat):
                if int(detected_id) != self.marker_id:
                    continue

                marker_corners = corners[i].reshape((4, 2))

                p0 = marker_corners[0]
                p1 = marker_corners[1]
                p2 = marker_corners[2]
                p3 = marker_corners[3]

                width_top = np.linalg.norm(p1 - p0)
                width_bottom = np.linalg.norm(p2 - p3)
                marker_width_px = (width_top + width_bottom) / 2.0

                if marker_width_px <= 1.0:
                    continue

                marker_center_x = float(np.mean(marker_corners[:, 0]))
                marker_center_y = float(np.mean(marker_corners[:, 1]))

                pixel_error_x = marker_center_x - (self.image_width / 2.0)

                # Simple distance estimate:
                # Z = real_marker_width * fx / marker_width_pixels
                distance = (self.marker_size * self.fx) / marker_width_px

                self.current_distance = float(distance)
                self.current_pixel_error_x = float(pixel_error_x)
                self.current_marker_center_x = float(marker_center_x)
                self.current_marker_center_y = float(marker_center_y)
                self.last_seen_time = self.get_clock().now()

                detected = True

                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = float(distance)
                pose.pose.position.y = float(-pixel_error_x / self.fx * distance)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                cv2.circle(
                    frame,
                    (int(marker_center_x), int(marker_center_y)),
                    5,
                    (0, 255, 0),
                    -1,
                )

                cv2.line(
                    frame,
                    (int(self.image_width / 2), 0),
                    (int(self.image_width / 2), int(self.image_height)),
                    (255, 0, 0),
                    2,
                )

                if self.use_tf_alignment:
                    status_text = f"Y:{self.y_ok} Yaw180:{self.yaw_ok}"
                else:
                    status_text = "TF disabled"

                text = (
                    f"ID {self.marker_id} "
                    f"dist={distance:.2f}m "
                    f"err={pixel_error_x:.1f}px "
                    f"{status_text}"
                )

                cv2.putText(
                    frame,
                    text,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 0),
                    2,
                )

                self.get_logger().info(
                    f"Detected marker {self.marker_id}: "
                    f"dist={distance:.2f} m, "
                    f"err_x={pixel_error_x:.1f} px",
                    throttle_duration_sec=0.5,
                )

                break

        if not detected:
            self.get_logger().warn(
                "Marker not detected in camera image",
                throttle_duration_sec=1.0,
            )

        try:
            debug_msg = self.cv2_to_ros_rgb_image(frame, msg.header)
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Debug image publish error: {e}")

    # ============================================================
    # TF alignment check
    #
    # Nav2 should already move robot to staging pose.
    # This node only checks:
    #   abs(robot_y - dock_y) <= y_threshold
    #   abs((dock_yaw + 180 deg) - robot_yaw) <= yaw_threshold
    # ============================================================
    def update_tf_alignment_state(self):
        self.tf_ok = False
        self.y_ok = False
        self.yaw_ok = False

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time(),
            )

            dock_tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.dock_frame,
                rclpy.time.Time(),
            )

        except Exception as e:
            self.get_logger().warn(
                f"Waiting for TF {self.global_frame}->{self.robot_frame} and "
                f"{self.global_frame}->{self.dock_frame}: {e}",
                throttle_duration_sec=2.0,
            )
            return

        robot_y = robot_tf.transform.translation.y
        dock_y = dock_tf.transform.translation.y

        robot_yaw = quaternion_to_yaw(robot_tf.transform.rotation)
        dock_yaw = quaternion_to_yaw(dock_tf.transform.rotation)

        y_error = dock_y - robot_y
        y_diff = abs(y_error)

        desired_robot_yaw = normalize_angle(dock_yaw + math.pi)
        yaw_error = normalize_angle(desired_robot_yaw - robot_yaw)
        yaw_diff = abs(yaw_error)

        self.current_robot_y = robot_y
        self.current_dock_y = dock_y
        self.current_y_error = y_error
        self.current_y_diff = y_diff

        self.current_robot_yaw = robot_yaw
        self.current_dock_yaw = dock_yaw
        self.current_desired_robot_yaw = desired_robot_yaw
        self.current_yaw_error_rad = yaw_error
        self.current_yaw_diff_rad = yaw_diff

        self.y_ok = y_diff <= self.y_threshold
        self.yaw_ok = yaw_diff <= self.yaw_threshold_rad
        self.tf_ok = True

        self.get_logger().info(
            f"TF check: "
            f"robot_y={robot_y:.3f}, dock_y={dock_y:.3f}, "
            f"y_error={y_error:.3f}, y_diff={y_diff:.3f}, "
            f"robot_yaw={math.degrees(robot_yaw):.1f} deg, "
            f"dock_yaw={math.degrees(dock_yaw):.1f} deg, "
            f"desired_robot_yaw={math.degrees(desired_robot_yaw):.1f} deg, "
            f"yaw_error={math.degrees(yaw_error):.1f} deg, "
            f"yaw_diff={math.degrees(yaw_diff):.1f} deg, "
            f"y_ok={self.y_ok}, yaw_180_ok={self.yaw_ok}",
            throttle_duration_sec=0.5,
        )

    # ============================================================
    # Helper functions
    # ============================================================
    def marker_is_fresh(self):
        if self.last_seen_time is None:
            return False

        age = self.get_clock().now() - self.last_seen_time
        return age <= Duration(seconds=0.5)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ============================================================
    # Main control loop
    #
    # This node does NOT align robot with TF using cmd_vel.
    # Nav2 must do the staging alignment first.
    #
    # This node only:
    #   1. checks TF Y/yaw condition
    #   2. checks ArUco detection
    #   3. performs final visual docking
    # ============================================================
    def control_loop(self):
        enable_docking = bool(self.get_parameter("enable_docking").value)

        if not enable_docking:
            return

        if self.docked:
            self.publish_stop()
            return

        # --------------------------------------------------------
        # Stage 1: Only check TF alignment
        # --------------------------------------------------------
        if self.use_tf_alignment:
            self.update_tf_alignment_state()

            if not self.tf_ok:
                self.get_logger().warn(
                    "TF not ready. Stop.",
                    throttle_duration_sec=1.0,
                )
                self.publish_stop()
                return

            if not (self.y_ok and self.yaw_ok):
                self.get_logger().warn(
                    "Robot is not aligned with ArUco TF yet. "
                    "Use Nav2 staging pose first. Stop.",
                    throttle_duration_sec=1.0,
                )
                self.publish_stop()
                return

        # --------------------------------------------------------
        # Stage 2: Final visual docking
        # --------------------------------------------------------
        if not self.marker_is_fresh():
            self.get_logger().warn(
                "TF aligned, but ArUco marker not detected. Stop.",
                throttle_duration_sec=1.0,
            )
            self.publish_stop()
            return

        self.run_final_docking_control()

    # ============================================================
    # Stage 2: final camera-based docking
    # ============================================================
    def run_final_docking_control(self):
        cmd = Twist()

        distance_error = self.current_distance - self.target_distance
        pixel_error_x = self.current_pixel_error_x

        if distance_error <= 0.03:
            self.get_logger().info("Docking success. Target distance reached.")
            self.docked = True
            self.publish_stop()
            return

        angular_z = -self.angular_kp * (
            pixel_error_x / (self.image_width / 2.0)
        )

        angular_z = clamp(
            angular_z,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        linear_x = 0.0

        if abs(pixel_error_x) < self.center_tolerance_px:
            linear_x = self.linear_kp * distance_error
            linear_x = clamp(
                linear_x,
                0.0,
                self.max_linear_speed,
            )

        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Final docking: "
            f"vx={linear_x:.3f}, wz={angular_z:.3f}, "
            f"dist={self.current_distance:.2f}, "
            f"target={self.target_distance:.2f}, "
            f"err_px={pixel_error_x:.1f}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSimpleDocking()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()