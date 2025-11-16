#!/usr/bin/env python3

"""A simple ArUco detector that publishes marker poses and TF frames."""

from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image
from tf_transformations import quaternion_from_matrix
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


def build_dictionary_map() -> Dict[str, int]:
    """Return available OpenCV ArUco dictionary constants keyed by name."""
    dictionary_names = [
        attr for attr in dir(cv2.aruco)
        if attr.startswith("DICT_") and isinstance(getattr(cv2.aruco, attr), int)
    ]
    return {name: getattr(cv2.aruco, name) for name in dictionary_names}


class ArucoDetector(Node):
    """Detect ArUco markers and publish poses, TFs and RViz markers."""

    def __init__(self) -> None:
        super().__init__("aruco_detector")

        self.bridge = CvBridge()

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("marker_id", -1)
        self.declare_parameter("aruco_dictionary", "DICT_5X5_100")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "aruco/debug")
        self.declare_parameter("marker_frame_prefix", "aruco_marker_")

        self.marker_size: float = float(self.get_parameter("marker_size").value)
        self.target_marker_id: int = int(self.get_parameter("marker_id").value)
        dictionary_name: str = str(self.get_parameter("aruco_dictionary").value)
        self.publish_debug_image: bool = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic: str = str(self.get_parameter("debug_image_topic").value)
        self.marker_frame_prefix: str = str(self.get_parameter("marker_frame_prefix").value)

        available_dictionaries = build_dictionary_map()
        if dictionary_name not in available_dictionaries:
            self.get_logger().warn(
                "Dictionary '%s' not available. Falling back to DICT_5X5_100.",
                dictionary_name,
            )
            dictionary_name = "DICT_5X5_100"
        self.dictionary = cv2.aruco.getPredefinedDictionary(available_dictionaries[dictionary_name])
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            # OpenCV < 4.7 provides a factory function instead of the constructor
            self.detector_params = cv2.aruco.DetectorParameters_create()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame: Optional[str] = None
        self._warned_missing_info = False

        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, qos)
        self.create_subscription(Image, image_topic, self._image_cb, qos)

        self.pose_pub = self.create_publisher(PoseStamped, "aruco/pose", 10)
        self.marker_pub = self.create_publisher(Marker, "aruco/marker", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        else:
            self.debug_pub = None

        target_description = (
            f"id={self.target_marker_id}"
            if self.target_marker_id >= 0
            else "all markers"
        )
        self.get_logger().info(
            f"ArUco detector active (target {target_description}, "
            f"size={self.marker_size:.3f} m, dict={dictionary_name})"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) == 0:
            if not self._warned_missing_info:
                self.get_logger().warn("Received empty camera info. Waiting for calibration data.")
                self._warned_missing_info = True
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        if len(msg.d) > 0:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        else:
            self.dist_coeffs = np.zeros(5, dtype=np.float64)

        self.camera_frame = msg.header.frame_id or "camera_optical_link"

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("Failed to convert image: %s", exc)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)

        if ids is None or len(ids) == 0:
            return

        detected_ids = ids.copy()
        ids = ids.flatten()

        if self.debug_pub is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, detected_ids)

        for marker_corners, marker_id in zip(corners, ids):
            if self.target_marker_id >= 0 and marker_id != self.target_marker_id:
                continue

            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = msg.header.frame_id or (self.camera_frame or "camera_optical_link")

            tvec = None
            rvec = None
            if self.camera_matrix is not None:
                try:
                    rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
                        marker_corners, self.marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn("Pose estimation failed: %s", exc)

            if tvec is None or rvec is None:
                if not self._warned_missing_info:
                    self.get_logger().warn(
                        "Skipping pose publishing because camera info is unavailable."
                    )
                    self._warned_missing_info = True
                continue

            pose.pose.position.x = float(tvec[0])
            pose.pose.position.y = float(tvec[1])
            pose.pose.position.z = float(tvec[2])

            rotation_matrix, _ = cv2.Rodrigues(rvec)
            homogeneous = np.eye(4, dtype=np.float64)
            homogeneous[:3, :3] = rotation_matrix
            quaternion = quaternion_from_matrix(homogeneous)
            pose.pose.orientation.x = float(quaternion[0])
            pose.pose.orientation.y = float(quaternion[1])
            pose.pose.orientation.z = float(quaternion[2])
            pose.pose.orientation.w = float(quaternion[3])

            self.pose_pub.publish(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = "aruco"
            marker.id = int(marker_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = 0.001
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            # marker.lifetime = Duration(seconds=0.2).to_msg()
            marker.lifetime = Duration(nanoseconds=200_000_000).to_msg()

            self.marker_pub.publish(marker)

            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = f"{self.marker_frame_prefix}{marker_id}"
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.translation.z = pose.pose.position.z
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

            if (
                self.debug_pub is not None
                and self.camera_matrix is not None
                and self.dist_coeffs is not None
                and rvec is not None
                and tvec is not None
            ):
                cv2.aruco.drawAxis(
                    frame,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5,
                )

        if self.debug_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
