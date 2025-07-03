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
from vgn.perception import UniformTSDFVolume
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2

from hsrlib.utils import description, utils
from robot_helpers.spatial import Transform as TransformHelper
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from tamlib.open3d import Open3D
from visualization_msgs.msg import MarkerArray


class CameraIntrinsic:
    def __init__(self, width, height, fx, fy, cx, cy):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy


class PoseEstimatorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.intrinsic = None
        self.description = description.load_robot_description()
        self.tamtf = Transform()
        self.depth_image = None

        # Library
        self.tam_bridge = TamCvBridge()
        self.open3d = Open3D()

        self.seg_sub = rospy.Subscriber("/cutie_tracking/result_segment", Image, self.seg_callback)
        self.depth_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth", CompressedImage, self.depth_callback)
        # self.depth_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_raw/compressed", CompressedImage, self.depth_callback)
        self.point_maker_pub = rospy.Publisher("/cutie_tracking/pose_estimator/maker", Marker, queue_size=1)
        self.bbox_maker_pub = rospy.Publisher("/cutie_tracking/pose_estimator/bbox", Marker, queue_size=1)
        self.pose_pub = rospy.Publisher("/cutie_tracking/pose_estimator/pose", Pose, queue_size=1)
        self.cutie_map_cloud_pub = rospy.Publisher("/cutie_tracking/pose_estimator/map_cloud", PointCloud2, queue_size=1)
        self.roi_pose_marker_pub = rospy.Publisher("/cutie_tracking/pose_estimator/roi", MarkerArray, queue_size=1)

        p_camera_info_topic = rospy.get_param("~camera_info_topic", "/hsrb/head_rgbd_sensor/depth_registered/camera_info")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "head_rgbd_sensor_rgb_frame")
        self.p_length = rospy.get_param("~length", 0.3)
        self.p_resolution = rospy.get_param("~resolution", 40)
        self.p_max_distance = rospy.get_param("~max_distance", -1)
        self.p_specific_id = rospy.get_param("~specific_id", "")
        self.p_task_frame_id = rospy.get_param("~task_frame_id", "pose_tracking")
        self.is_pub_bbox = rospy.get_param("~is_pub_bbox", True)

        self.camera_info = rospy.wait_for_message(p_camera_info_topic, CameraInfo)
        self.set_intrinsic(self.camera_info)
        self.set_camera_model(self.camera_info)

        rospy.loginfo("PoseEstimatorNode initialized (no sync). Waiting for messages...")


    def from_pose_msg(self, msg):
        translation = self.from_vector3_msg(msg.position)
        rotation = self.from_quat_msg(msg.orientation)
        return TransformHelper(rotation, translation)

    @staticmethod
    def from_quat_msg(msg):
        return Rotation.from_quat([msg.x, msg.y, msg.z, msg.w])

    @staticmethod
    def from_vector3_msg(msg):
        return np.r_[msg.x, msg.y, msg.z]

    @staticmethod
    def to_point_msg(point):
        msg = Point()
        msg.x = point[0]
        msg.y = point[1]
        msg.z = point[2]
        return msg

    @staticmethod
    def to_quat_msg(orientation):
        quat = orientation.as_quat()
        msg = Quaternion()
        msg.x = quat[0]
        msg.y = quat[1]
        msg.z = quat[2]
        msg.w = quat[3]
        return msg

    def to_pose_msg(self, transform):
        msg = Pose()
        msg.position = self.to_point_msg(transform.translation)
        msg.orientation = self.to_quat_msg(transform.rotation)
        return msg

    @staticmethod
    def to_vector3_msg(vector3):
        msg = Vector3()
        msg.x = vector3[0]
        msg.y = vector3[1]
        msg.z = vector3[2]
        return msg

    @staticmethod
    def to_color_msg(color):
        msg = ColorRGBA()
        msg.r = color[0]
        msg.g = color[1]
        msg.b = color[2]
        msg.a = color[3] if len(color) == 4 else 1.0
        return msg

    @staticmethod
    def to_cloud_msg(frame, points, colors=None, intensities=None, distances=None):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame

        msg.height = 1
        msg.width = points.shape[0]
        msg.is_bigendian = False
        msg.is_dense = False

        msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
        ]
        msg.point_step = 12
        data = points

        if colors is not None:
            raise NotImplementedError
        elif intensities is not None:
            msg.fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))
            msg.point_step += 4
            data = np.hstack([points, intensities])
        elif distances is not None:
            msg.fields.append(PointField("distance", 12, PointField.FLOAT32, 1))
            msg.point_step += 4
            data = np.hstack([points, distances])

        msg.row_step = msg.point_step * points.shape[0]
        msg.data = data.astype(np.float32).tostring()

        return msg

    @staticmethod
    def identity():
        return TransformHelper.identity()

    @staticmethod
    def box_lines(lower, upper):
        x_l, y_l, z_l = lower
        x_u, y_u, z_u = upper
        return [
            ([x_l, y_l, z_l], [x_u, y_l, z_l]),
            ([x_u, y_l, z_l], [x_u, y_u, z_l]),
            ([x_u, y_u, z_l], [x_l, y_u, z_l]),
            ([x_l, y_u, z_l], [x_l, y_l, z_l]),
            ([x_l, y_l, z_u], [x_u, y_l, z_u]),
            ([x_u, y_l, z_u], [x_u, y_u, z_u]),
            ([x_u, y_u, z_u], [x_l, y_u, z_u]),
            ([x_l, y_u, z_u], [x_l, y_l, z_u]),
            ([x_l, y_l, z_l], [x_l, y_l, z_u]),
            ([x_u, y_l, z_l], [x_u, y_l, z_u]),
            ([x_u, y_u, z_l], [x_u, y_u, z_u]),
            ([x_l, y_u, z_l], [x_l, y_u, z_u]),
        ]

    def create_marker(self, type, frame, pose, scale=None, color=None, ns="", id=0):
        if scale is None:
            scale = [1, 1, 1]
        elif np.isscalar(scale):
            scale = [scale, scale, scale]
        if color is None:
            color = (1, 1, 1)
        msg = Marker()
        msg.header.frame_id = frame
        msg.header.stamp = rospy.Time()
        msg.ns = ns
        msg.id = id
        msg.type = type
        msg.action = Marker.ADD
        msg.pose = self.to_pose_msg(pose)
        msg.scale = self.to_vector3_msg(scale)
        msg.color = self.to_color_msg(color)
        return msg

    def create_line_list_marker(self, frame, pose, scale, color, lines, ns="", id=0):
        marker = self.create_marker(Marker.LINE_LIST, frame, pose, scale, color, ns, id)
        marker.points = [self.to_point_msg(point) for line in lines for point in line]
        return marker


    def create_line_strip_marker(self, frame, pose, scale, color, points, ns="", id=0):
        marker = self.create_marker(Marker.LINE_STRIP, frame, pose, scale, color, ns, id)
        marker.points = [self.to_point_msg(point) for point in points]
        return marker


    def create_grasp_marker(frame, grasp, color, ns, id=0, depth=0.05, radius=0.005):
        # Faster grasp marker using Marker.LINE_LIST
        pose, w, d, scale = grasp.pose, grasp.width, depth, [radius, 0.0, 0.0]
        w *= 0.135 / 1  # width=1 -> 13.5cm
        points = [[0, -w / 2, d], [0, -w / 2, 0], [0, w / 2, 0], [0, w / 2, d]]
        return self.create_line_strip_marker(frame, pose, scale, color, points, ns, id)


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

    def set_intrinsic(self, camera_info: CameraInfo) -> None:
        self.intrinsic = CameraIntrinsic(
            camera_info.width,
            camera_info.height,
            camera_info.K[0],
            camera_info.K[4],
            camera_info.K[2],
            camera_info.K[5],
        )

    def marker_publisher(self, pose):
        marker = Marker()
        marker.header.frame_id = self.frame_id
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
        self.point_maker_pub.publish(marker)

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
        self.depth_image_float = (
                self.tam_bridge.compressed_imgmsg_to_depth(msg).astype(np.float32)
                * 0.001
            )

        self.depth_image = self.tam_bridge.compressed_imgmsg_to_depth(msg).astype(np.float32)

    def make_3d_bbox_from_pcd2msg(self, msg: PointCloud2):
        pc = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        ])

        if pc.shape[0] == 0:
            rospy.logwarn("Empty point cloud.")
            return

        # Open3DでBBOX計算
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc)
        aabb = pcd.get_axis_aligned_bounding_box()

        # BBOXをMarkerとしてRVizに配信
        marker = Marker()
        marker.header.frame_id = self.p_task_frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bbox"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        center = aabb.get_center()
        extent = aabb.get_extent()

        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0  # no rotation

        marker.scale.x = extent[0]
        marker.scale.y = extent[1]
        marker.scale.z = extent[2]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.lifetime = rospy.Duration(0.1)
        self.bbox_maker_pub.publish(marker)        

    def roi(self, frame, size):
        pose = self.identity()
        scale = [size * 0.005, 0.0, 0.0]
        color = [0.5, 0.5, 0.5]
        lines = self.box_lines(np.full(3, 0), np.full(3, size))
        msg = self.create_line_list_marker(frame, pose, scale, color, lines, ns="roi")
        marker_arr = MarkerArray()
        marker_arr.markers.append(msg)
        self.roi_pose_marker_pub.publish(marker_arr)

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

        depth = self.depth_image_float
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

        if self.is_pub_bbox:
            pose_on_map.position.x -= self.p_length / 2.0
            pose_on_map.position.y -= self.p_length / 2.0
            pose_on_map.position.z -= self.p_length / 2.0
            if pose_on_map.position.z < 0.0:
                pose_on_map.position.z = 0.0
            pose_on_map.orientation = Quaternion(0, 0, 0, 1)

            pose_on_camera = self.tamtf.get_pose_with_offset(
                self.description.frame.rgbd,
                self.description.frame.map,
                pose_on_map,
                "pose_tracking_camera",
            )

            self.tamtf.send_static_transform(
                self.p_task_frame_id, self.camera_frame_id, pose_on_camera
            )
            self.roi(self.p_task_frame_id, self.p_length)

            # Get map cloud
            depth[mask < 127] = 0
            extrinsic = self.from_pose_msg(pose_on_camera)
            tsdf = UniformTSDFVolume(self.p_length, self.p_resolution)
            tsdf.integrate(depth, self.intrinsic, extrinsic)
            map_cloud = tsdf.get_map_cloud()
            # _, idx = map_cloud.remove_radius_outlier(nb_points=16, radius=0.02)
            # map_cloud = map_cloud.select_by_index(idx)
            points = np.ascontiguousarray(map_cloud.points)
            msg = self.to_cloud_msg(self.p_task_frame_id, points, distances=None)
            self.cutie_map_cloud_pub.publish(msg)
            self.make_3d_bbox_from_pcd2msg(msg)

if __name__ == "__main__":
    rospy.init_node("pose_estimator_node")
    node = PoseEstimatorNode()
    rospy.spin()
