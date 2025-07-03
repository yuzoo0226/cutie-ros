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
from cutie_ros.srv import StartTracking, StartTrackingRequest, StopTracking, StopTrackingRequest, StopTrackingResponse
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest


class CutieTrackingUtils:
    def __init__(self):
        self.bridge = CvBridge()

        rospy.wait_for_service("/cutie/start_tracking")
        self.start_service = rospy.ServiceProxy("/cutie/start_tracking", StartTracking)
        self.stop_service = rospy.ServiceProxy("/cutie/stop_tracking", StopTracking)
        self.clear_service = rospy.ServiceProxy("/cutie/clear_memory", Trigger)

    # TODO(yano): IDの指定に対応する
    def detection_based_tracking(self, msg: ObjectDetection):
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

        req = StartTrackingRequest()
        req.images = image_msgs
        req.masks = mask_msgs

        try:
            resp = self.start_service(req)
            rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def saved_mask_based_tracking(self, object_name: str):
        req = StartTrackingRequest()
        req.object_name = object_name

        try:
            resp = self.start_service(req)
            rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        return resp

    def stop_tracking(self):
        req = StopTrackingRequest()
        resp = self.stop_service(req)
        rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        return resp

    def clear_memory(self):
        req = TriggerRequest()
        resp = self.clear_service(req)
        rospy.loginfo(f"Success: {resp.success}, Message: {resp.message}")
        return resp


if __name__ == "__main__":
    rospy.init_node("test_start_tracking_node")
    node = CutieTrackingUtils()
    node.saved_mask_based_tracking(object_name="dishwasher")
