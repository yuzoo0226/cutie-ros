#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import glob
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tam_object_detection.msg import ObjectDetection
from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceResponse
from cutie_ros.srv import StartTracking, StartTrackingRequest


class StartTrackingTestClient:
    def __init__(self):
        rospy.init_node("test_start_tracking_node")
        self.bridge = CvBridge()

        rospy.wait_for_service("start_tracking")
        self.start_service = rospy.ServiceProxy("start_tracking", StartTracking)
        self.detection_service = rospy.ServiceProxy("tam_object_detection", ObjectDetectionService)
        self.sub_detection_result = rospy.Subscriber("/hsr_head_rgbd/object_detection/detection", ObjectDetection, self.cb_detection_result)
        self.run_enable = True

    def cb_detection_result(self, msg: ObjectDetection):
        """_summary_

        Args:
            msg (ObjectDetection): _description_
        """
        mask_msgs = []
        image_msgs = []

        masks_msg = msg.segments
        msg_bgr = msg.rgb

        if len(masks_msg) < 1:
            return

        np_bgr: np.ndarray = np.frombuffer(msg_bgr.data, dtype=np.uint8)
        cv_bgr: np.ndarray = cv2.imdecode(np_bgr, cv2.IMREAD_COLOR)
        msg_bgr = self.bridge.cv2_to_imgmsg(cv_bgr, encoding="bgr8")

        for msg_mask in masks_msg:
            np_mask: np.ndarray = np.frombuffer(msg_mask.data, dtype=np.uint8)
            cv_mask: np.ndarray = cv2.imdecode(np_mask, cv2.IMREAD_GRAYSCALE)
            msg_mask = self.bridge.cv2_to_imgmsg(cv_mask, encoding="mono8")

            mask_msgs.append(msg_mask)
            image_msgs.append(msg_bgr)

        if self.run_enable:
            self.call_start_tracking(image_msgs, mask_msgs)
            self.run_enable = False

        cv2.imshow("segment_mask", cv_mask)
        cv2.waitKey(1)

    def call_start_tracking(self, image_msgs, mask_msgs):
        req = StartTrackingRequest()
        req.images = image_msgs
        req.masks = mask_msgs

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
    node.spin()
