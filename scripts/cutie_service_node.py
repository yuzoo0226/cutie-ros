import os
import sys
import cv2
import rospy
import torch
import logging
import threading
import numpy as np
from PIL import PILImage
from argparse import ArgumentParser
from torchvision.transforms.functional import to_tensor

from cutie.inference.inference_core import InferenceCore
from cutie.utils.get_default_model import get_default_model

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from hkchengrex_Cutie.srv import StartTracking, StartTrackingResponse, StopTracking, StopTrackingResponse


class TrackingNode:
    def __init__(self):
        self.is_tracking = False
        self.lock = threading.Lock()

        self.cutie = get_default_model()
        self.processor = InferenceCore(self.cutie, cfg=self.cutie.cfg)
        self.processor.max_internal_size = 480

        self.result_pub = rospy.Publisher("/cutie_tracking/result_image", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/hsrb/hand_camera/image_raw/compressed", CompressedImage, self.sub_hand_bgr)

        self.start_service = rospy.Service("start_tracking", StartTracking, self.handle_start)
        self.stop_service = rospy.Service("stop_tracking", StopTracking, self.handle_stop)

        self.tracking_thread = None
        rospy.loginfo("TrackingNode initialized.")

    def sub_hand_bgr(self, msg: CompressedImage):
        # Decode the compressed image to OpenCV format
        np_arr: np.ndarray = np.frombuffer(msg.data, dtype=np.uint8)
        cv_bgr: np.ndarray = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)

        self.target_pil_rgb = PILImage.fromarray(cv_rgb)

    def handle_start(self, req):
        with self.lock:
            if self.is_tracking:
                return StartTrackingResponse(False, "Tracking is already running.")
            self.is_tracking = True

        self.images = req.images
        self.masks = req.masks
        self.timestep = 0

        for idx in len(self.images):
            cv_bgr = self.images[idx]
            cv_mask = self.masks[idx]

            # OpenCV(BGR) → RGB
            cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
            cv_mask = cv2.cvtColor(cv_mask, cv2.COLOR_BGR2RGB)

            # NumPy配列 → Pillow画像
            pil_rgb = PILImage.fromarray(cv_rgb)
            pil_mask = PILImage.fromarray(cv_mask)

            assert pil_mask.mode in ['L', 'P']

            # palette is for visualization
            self.palette = pil_mask.getpalette()
            self.objects = np.unique(np.array(pil_mask))
            # background "0" does not count as an object
            self.objects = self.objects[self.objects != 0].tolist()

            torch_rgb = to_tensor(pil_rgb).cuda().float()
            torch_mask = torch.from_numpy(np.array(pil_mask)).cuda()

            _ = self.processor.step(torch_rgb, torch_mask, objects=self.objects)

        self.tracking_thread = threading.Thread(target=self.tracking_loop)
        self.tracking_thread.start()
        rospy.loginfo("Tracking started with {} images.".format(len(self.images)))
        return StartTrackingResponse(True, "Tracking started.")

    def handle_stop(self, req):
        with self.lock:
            if not self.is_tracking:
                return StopTrackingResponse(False, "Tracking not running.")
            self.is_tracking = False
        rospy.loginfo("Tracking stopped by user.")
        return StopTrackingResponse(True, "Tracking stopped.")

    @torch.inference_mode()
    @torch.cuda.amp.autocast()
    def tracking_loop(self):
        while not rospy.is_shutdown():
            with self.lock:
                if not self.is_tracking:
                    break

            target_image = to_tensor(self.target_pil_rgb).cuda().float()
            output_prob = self.processor.step(target_image)  # inference
            mask = self.processor.output_prob_to_mask(output_prob)

            # visualize prediction
            estimation_mask = PILImage.fromarray(mask.cpu().numpy().astype(np.uint8))
            cv_rgb = cv2.cvtColor(np.array(estimation_mask), cv2.COLOR_RGB2BGR)

            # RGB → BGR（OpenCVはBGRを使用）
            cv_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)

            ros_img_msg = self.bridge.cv2_to_imgmsg(cv_bgr, encoding="bgr8")
            self.result_pub.publish(ros_img_msg)


if __name__ == "__main__":
    rospy.init_node("tracking_node")
    node = TrackingNode()
    rospy.spin()
