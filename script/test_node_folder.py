#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import glob
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from cutie_ros.srv import StartTracking, StartTrackingRequest


class StartTrackingTestClient:
    def __init__(self):
        rospy.init_node("test_start_tracking_node")
        self.bridge = CvBridge()

        rospy.wait_for_service("/cutie/start_tracking")
        self.start_service = rospy.ServiceProxy("/cutie/start_tracking", StartTracking)

        # image_paths = sorted(glob.glob("./io/test_images/images/*.png"))
        # mask_paths = sorted(glob.glob("./io/test_images/masks/*.png"))

        # assert len(image_paths) == len(mask_paths), "Unmatched images and masks number"

        # self.images = []
        # self.masks = []

        # for img_path, mask_path in zip(image_paths, mask_paths):
        #     cv_img = cv2.imread(img_path)
        #     cv_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

        #     if cv_img is None or cv_mask is None:
        #         rospy.logerr(f"Failed to load {img_path} or {mask_path}")
        #         continue

        #     cv_mask_rgb = cv2.cvtColor(cv_mask, cv2.COLOR_GRAY2RGB)

        #     ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        #     ros_mask = self.bridge.cv2_to_imgmsg(cv_mask_rgb, encoding="rgb8")

        #     self.images.append(ros_img)
        #     self.masks.append(ros_mask)

    def call_start_tracking(self, object_name: str):
        req = StartTrackingRequest()
        req.object_name = object_name
        # req.images = self.images
        # req.masks = self.masks

        try:
            resp = self.start_service(req)
            rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = StartTrackingTestClient()
    node.call_start_tracking(object_name="door_knob")
    node.spin()
