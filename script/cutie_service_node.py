#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import cv2
import rospy
import torch
import roslib
import logging
import threading
import numpy as np
from cv_bridge import CvBridge
from PIL import Image as PILImage
from torchvision.transforms.functional import to_tensor

from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool
from cutie_ros.srv import StartTracking, StartTrackingResponse, StopTracking, StopTrackingResponse
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolRequest, SetBoolResponse


class TrackingNode:
    def __init__(self):
        self.is_tracking = False
        self.use_cv2_window = False
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.cutie = get_default_model()
        self.processor = InferenceCore(self.cutie, cfg=self.cutie.cfg)
        self.processor.max_internal_size = 480

        self.result_bgr_pub = rospy.Publisher("/cutie_tracking/result_bgr", Image, queue_size=1)
        self.result_segment_pub = rospy.Publisher("/cutie_tracking/result_segment", Image, queue_size=1)
        self.status_string_pub = rospy.Publisher("/cutie_tracking/tracking_status", Bool, queue_size=1)

        # self.image_sub = rospy.Subscriber("/hsrb/hand_camera/image_raw/compressed", CompressedImage, self.sub_hand_bgr)
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw/compressed", CompressedImage, self.sub_hand_bgr)

        self.cutie_package_dir = roslib.packages.get_pkg_dir("cutie_ros")
        self.start_service = rospy.Service("/cutie/start_tracking", StartTracking, self.handle_start)
        self.stop_service = rospy.Service("/cutie/stop_tracking", StopTracking, self.handle_stop)
        self.clear_service = rospy.Service("/cutie/clear_memory", Trigger, self.handle_clear)

        self.run_enable = True
        self.run_enable_service = rospy.Service("/cutie/run_enable", SetBool, self.handle_run_enable)

        self.tracking_thread = None
        self.target_pil_rgb = None
        rospy.loginfo("TrackingNode initialized.")

    def handle_run_enable(self, req: SetBoolRequest) -> SetBoolResponse:
        """
        Handle the run enable service to start or stop tracking.
        """
        self.run_enable = req.data
        if self.run_enable:
            rospy.loginfo("Tracking enabled.")
        else:
            rospy.loginfo("Tracking disabled.")
        return SetBoolResponse(success=True, message="Run enable set to {}".format(req.data))

    def sub_hand_bgr(self, msg: CompressedImage):
        # Decode the compressed image to OpenCV format
        np_arr: np.ndarray = np.frombuffer(msg.data, dtype=np.uint8)
        cv_bgr: np.ndarray = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        self.target_pil_rgb = PILImage.fromarray(cv_rgb)

    @staticmethod
    def cv2_to_pillow_image(cv_image: np.ndarray) -> PILImage:
        if len(cv_image.shape) == 2:
            return PILImage.fromarray(cv_image, mode='L')
        elif len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            return PILImage.fromarray(rgb_image)
        else:
            raise ValueError("Unsupport Image type")

    @staticmethod
    def pillow_to_cv2(pil_img: PILImage) -> np.ndarray:
        """Convert Pillow image to OpenCV BGR numpy array"""
        img_np = np.array(pil_img)
        if pil_img.mode == "RGBA":
            return cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)
        elif pil_img.mode == "RGB":
            return cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
        elif pil_img.mode == "L":
            return img_np
        else:
            raise ValueError(f"Unsupported image mode: {pil_img.mode}")

    def handle_clear(self, req):
        self.processor.clear_memory()
        rospy.loginfo("Clear tracking object memory.")
        return TriggerResponse(True, "Clear tracking object memory.")

    def handle_start(self, req):
        with self.lock:
            if self.is_tracking:
                return StartTrackingResponse(False, "Tracking is already running.")
            self.is_tracking = True

        rospy.set_param("/cutie/task_frame_id", req.object_name)

        self.req = req
        self.images = req.images
        self.masks = req.masks
        self.obj_name = req.object_name

        if self.obj_name is not None:
            cv_bgr = cv2.imread(os.path.join(self.cutie_package_dir, f"io/images/{self.obj_name}.png"), 1)
            cv_mask = cv2.imread(os.path.join(self.cutie_package_dir, f"io/masks/{self.obj_name}.png"), 0)
            cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)

            pil_rgb = PILImage.fromarray(cv_rgb)
            pil_mask = PILImage.fromarray(cv_mask, mode="L")

            # palette is for visualization
            self.palette = pil_mask.getpalette()
            self.objects = np.unique(np.array(pil_mask))
            self.objects = self.objects[self.objects != 0].tolist()

            torch_rgb = to_tensor(pil_rgb).cuda().float()
            torch_mask = torch.from_numpy(np.array(pil_mask)).cuda()

            _ = self.processor.step(torch_rgb, torch_mask, objects=self.objects)
            rospy.loginfo(f"Tracking started with {self.obj_name} images.")

        for idx in range(len(self.images)):
            print(idx)
            msg_bgr = self.images[idx]
            msg_mask = self.masks[idx]

            cv_bgr = self.bridge.imgmsg_to_cv2(msg_bgr, "bgr8")
            cv_mask = self.bridge.imgmsg_to_cv2(msg_mask, "mono8")

            # OpenCV(BGR) → RGB
            cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)

            # NumPy配列 → Pillow画像
            pil_rgb = PILImage.fromarray(cv_rgb)
            pil_mask = PILImage.fromarray(cv_mask, mode="L")

            assert pil_mask.mode in ['L', 'P']

            # palette is for visualization
            self.palette = pil_mask.getpalette()
            self.objects = np.unique(np.array(pil_mask))
            self.objects = self.objects[self.objects != 0].tolist()

            torch_rgb = to_tensor(pil_rgb).cuda().float()
            torch_mask = torch.from_numpy(np.array(pil_mask)).cuda()

            _ = self.processor.step(torch_rgb, torch_mask, objects=self.objects)
            rospy.loginfo("Tracking started with {} images.".format(len(self.images)))
            break

        self.tracking_thread = threading.Thread(target=self.tracking_loop)
        self.tracking_thread.start()
        # rospy.loginfo("Tracking started with {} images.".format(len(self.images)))
        return StartTrackingResponse(True, "Tracking started.")

    def handle_stop(self, req):
        with self.lock:
            if not self.is_tracking:
                return StopTrackingResponse(False, "Tracking not running.")
            self.is_tracking = False
        rospy.loginfo("Tracking stopped by user.")
        return StopTrackingResponse(True, "Tracking stopped.")

    def make_overlay_image(self, pillow_mask, pillow_bgr, color=(0, 0, 255), alpha=0.5):
        cv_bgr = self.pillow_to_cv2(pillow_bgr)
        cv_mask = self.pillow_to_cv2(pillow_mask)

        if len(cv_mask.shape) == 3 and cv_mask.shape[2] == 3:
            cv_mask = cv2.cvtColor(cv_mask, cv2.COLOR_BGR2GRAY)
        else:
            cv_mask = cv_mask

        # マスクを2値化
        mask_bin = (cv_mask > 0).astype(np.uint8)

        # カラーマスクを作成
        color_mask = np.zeros_like(cv_bgr)
        color_mask[:] = color

        # マスク領域だけ色を合成
        overlay = cv_bgr.copy()
        overlay[mask_bin == 1] = cv2.addWeighted(
            cv_bgr[mask_bin == 1], 1 - alpha,
            color_mask[mask_bin == 1], alpha,
            0
        )

        return overlay

    @torch.inference_mode()
    @torch.cuda.amp.autocast()
    def tracking_loop(self):
        while not rospy.is_shutdown():
            with self.lock:
                if not self.is_tracking:
                    break
            
            if self.run_enable is False:
                rospy.logdebug("Tracking is not enabled, waiting for start command.")
                rospy.sleep(0.5)
                continue

            if self.target_pil_rgb is None:
                rospy.logwarn("No target image available for tracking.")
                rospy.sleep(1)
                continue

            target_image = to_tensor(self.target_pil_rgb).cuda().float()
            output_prob = self.processor.step(target_image)  # inference
            mask = self.processor.output_prob_to_mask(output_prob)

            # visualize prediction
            estimation_mask = PILImage.fromarray(mask.cpu().numpy().astype(np.uint8))
            pil_rgb = estimation_mask.convert("RGB")
            cv_rgb = np.array(pil_rgb)
            cv_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)

            try:
                overlay_mask = self.make_overlay_image(pillow_mask=pil_rgb, pillow_bgr=self.target_pil_rgb)
                segment_img_msg = self.bridge.cv2_to_imgmsg(cv_bgr, encoding="bgr8")
                overlay_img_msg = self.bridge.cv2_to_imgmsg(overlay_mask, encoding="bgr8")
                self.result_segment_pub.publish(segment_img_msg)
                self.result_bgr_pub.publish(overlay_img_msg)
                self.status_string_pub.publish(Bool(data=True))
            except TypeError as e:
                # self.handle_clear(None)
                # rospy.sleep(1)
                # self.handle_stop(None)
                # rospy.sleep(1)
                # self.handle_start(self.req)  # Restart tracking if TypeError occurs
                # rospy.sleep(3)
                self.status_string_pub.publish(Bool(data=False))
                rospy.logwarn(f"{e}: tracking failed")
            except UnboundLocalError as e:
                rospy.logwarn(f"{e}: tracking failed, probably no object detected")

            if self.use_cv2_window:
                cv2.imshow("tracking_image", overlay_mask)
                cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("tracking_node")
    node = TrackingNode()
    rospy.spin()
