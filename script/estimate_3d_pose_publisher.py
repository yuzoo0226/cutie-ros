#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import torch
from torch import Tensor
from cv_bridge import CvBridge
from tamlib.tf import Transform, euler2quaternion
from tamlib.cv_bridge import CvBridge as TamCvBridge
from typing import Dict, List, Optional, Tuple
from image_geometry import PinholeCameraModel

from hsrlib.utils import description, utils
from tamlib.tf import Transform
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from tamlib.open3d import Open3D
from visualization_msgs.msg import MarkerArray


class PoseEstimatorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.intrinsic = None
        self.description = description.load_robot_description()
        self.tamtf = Transform()
        self.depth_image = None

        # Library
        self.tam_bridge = TamCvBridge()
        self.tamtf = Transform()
        self.open3d = Open3D()

        self.seg_sub = rospy.Subscriber("/cutie_tracking/result_segment", Image, self.seg_callback)
        self.depth_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth", CompressedImage, self.depth_callback)
        # self.depth_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw/compressed", CompressedImage, self.depth_callback)
        self.maker_pub = rospy.Publisher("/cutie_tracking/pose_estimator/maker", Marker, queue_size=1)
        self.pose_pub = rospy.Publisher("/cutie_tracking/pose_estimator/pose", Pose, queue_size=1)

        p_camera_info_topic = rospy.get_param("~camera_info_topic", "/hsrb/head_rgbd_sensor/depth_registered/camera_info")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "head_rgbd_sensor_rgb_frame")
        self.p_length = rospy.get_param("~length", 0.3)
        self.p_resolution = rospy.get_param("~resolution", 40)
        self.p_max_distance = rospy.get_param("~max_distance", -1)
        self.p_specific_id = rospy.get_param("~specific_id", "")

        self.camera_info = rospy.wait_for_message(p_camera_info_topic, CameraInfo)
        self.set_camera_model(self.camera_info)

        rospy.loginfo("PoseEstimatorNode initialized (no sync). Waiting for messages...")

    @staticmethod
    def get_bbox_from_mask(mask_img):
        """
        mask_img: 2D numpy array (0 or 255, or 0-1) のマスク画像
        return: (x_min, y_min, x_max, y_max)
        """
        # もし0-1のマスクなら255に変換
        if mask_img.max() <= 1:
            mask_img = (mask_img * 255).astype(np.uint8)

        # 輪郭抽出
        contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Found {len(contours)} contours.")
        # print(contours)

        if len(contours) == 0:
            return None  # 物体がない場合

        # 一番大きい領域を選択（単一物体前提）
        largest_contour = max(contours, key=cv2.contourArea)

        # BBOX計算
        x, y, w, h = cv2.boundingRect(largest_contour)
        x_min = x
        y_min = y
        x_max = x + w
        y_max = y + h

        return (x_min, y_min, x_max, y_max)

    def marker_publisher(self, pose):
        marker = Marker()
        marker.header.frame_id = "map"    # ここは適切なフレームIDに変更してください
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y
        marker.pose.position.z = pose.position.z
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.maker_pub.publish(marker)

    def get_3d_poses(self, depth: np.ndarray, bboxes: Tensor) -> List[Optional[Pose]]:
        """カメラ座標系での三次元座標を取得する（物体中心）

        Args:
            depth (np.ndarray): Depth画像．
            bboxes (Tensor): BBox情報．

        Returns:
            List[Optional[Pose]]: 各オブジェクトの3次元座標．
                計算できなかった場合，Noneが格納される．
        """
        poses: List[Optional[Pose]] = []
        for box in bboxes:
            w, h = box[2] - box[0], box[3] - box[1]
            cx, cy = int(box[0] + w / 2), int(box[1] + h / 2)
            crop_depth = depth[cy - 2 : cy + 3, cx - 2 : cx + 3] * 0.001
            flat_depth = crop_depth[crop_depth != 0].flatten()
            if len(flat_depth) == 0:
                poses.append(None)
                continue
            mean_depth = np.mean(flat_depth)
            uv = list(self.camera_model.projectPixelTo3dRay((cx, cy)))
            uv[:] = [x / uv[2] for x in uv]
            uv[:] = [x * mean_depth for x in uv]
            if self.p_max_distance < 0 or (
                self.p_max_distance > 0 and self.p_max_distance > uv[2]
            ):
                poses.append(Pose(Point(*uv), Quaternion(0, 0, 0, 1)))
            else:
                poses.append(None)
        return poses

    def set_camera_model(self, camera_info: CameraInfo) -> None:
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)
        self.camera_frame_id = camera_info.header.frame_id

    def depth_callback(self, msg):
        self.depth_image = self.tam_bridge.compressed_imgmsg_to_depth(msg).astype(np.float32)

        # np_depth = np.frombuffer(msg.data, dtype=np.uint8)
        # cv_depth = cv2.imdecode(np_depth, cv2.IMREAD_UNCHANGED)
        # self.depth_image = cv_depth

    def seg_callback(self, msg):
        if self.depth_image is None:
            rospy.logwarn("Depth image not yet available.")
            return

        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        bbox_tuple = self.get_bbox_from_mask(mask_img=mask)
        if bbox_tuple is None:
            rospy.logwarn("No valid mask found.")
            return

        poses = self.get_3d_poses(
            depth=self.depth_image,
            bboxes=torch.tensor([bbox_tuple], dtype=torch.float32) ,
        )

        center_pose = poses[0]  # Assuming single object detection

        if center_pose is None:
            rospy.logwarn("No valid pose found for the detected object.")
            return

        pose_on_map = self.tamtf.get_pose_with_offset(
            self.description.frame.map,
            self.description.frame.rgbd,
            center_pose,
            "pose_tracking",
        )

        self.pose_pub.publish(center_pose)
        self.marker_publisher(pose_on_map)


if __name__ == "__main__":
    rospy.init_node("pose_estimator_node")
    node = PoseEstimatorNode()
    rospy.spin()
