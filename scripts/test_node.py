#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import glob
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cutie_ros.srv import StartTracking, StartTrackingRequest


class StartTrackingTestClient:
    def __init__(self):
        rospy.init_node("test_start_tracking_node")
        self.bridge = CvBridge()

        # サービスが利用可能になるのを待機
        rospy.wait_for_service("start_tracking")
        self.start_service = rospy.ServiceProxy("start_tracking", StartTracking)

        # 画像とマスクのパス（例として ./test_images/）
        image_paths = sorted(glob.glob("./io/test_images/images/*.png"))
        mask_paths = sorted(glob.glob("./io/test_images/masks/*.png"))

        assert len(image_paths) == len(mask_paths), "Unmatched images and masks number"

        # 読み込みと変換
        self.images = []
        self.masks = []

        for img_path, mask_path in zip(image_paths, mask_paths):
            cv_img = cv2.imread(img_path)
            cv_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

            if cv_img is None or cv_mask is None:
                rospy.logerr(f"Failed to load {img_path} or {mask_path}")
                continue

            # 必要に応じてマスクを3チャンネルに変換
            cv_mask_rgb = cv2.cvtColor(cv_mask, cv2.COLOR_GRAY2RGB)

            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            ros_mask = self.bridge.cv2_to_imgmsg(cv_mask_rgb, encoding="rgb8")

            self.images.append(ros_img)
            self.masks.append(ros_mask)

    def call_start_tracking(self):
        req = StartTrackingRequest()
        req.images = self.images
        req.masks = self.masks

        try:
            resp = self.start_service(req)
            rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    client = StartTrackingTestClient()
    client.call_start_tracking()
