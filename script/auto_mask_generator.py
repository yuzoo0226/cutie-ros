#!/usr/bin/env python3
import os
import sys
import cv2
import torch
import rospy
import roslib
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from mobile_sam import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor


class AutoMaskGenerator:
    def __init__(self):
        rospy.init_node("image_click_viewer")

        model_type = "vit_t"
        self.package_dir = roslib.packages.get_pkg_dir("cutie_ros")
        sam_checkpoint = os.path.join(self.package_dir, "io/weights/mobile_sam.pt")

        # self.encoder_path = {
        #     'efficientvit_l2': os.path.join(package_dir, "io/weights/l2.pt"),
        #     'tiny_vit': os.path.join(package_dir, "io/weights/mobile_sam.pt"),
        #     'sam_vit_h': os.path.join(package_dir, "io/weights/sam_vit_h.pt"),
        # }

        device = "cuda" if torch.cuda.is_available() else "cpu"

        self.mobile_sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
        self.mobile_sam.to(device=device)
        self.mobile_sam.eval()
        self.predictor = SamPredictor(self.mobile_sam)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/hsrb/hand_camera/image_raw/compressed", CompressedImage, self.image_callback)
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw/compressed", CompressedImage, self.image_callback)

        # self.cv_image = None
        self.right_drag_start = None
        self.right_drag_end = None

        # cv2.startWindowThread()
        # cv2.namedWindow("Live Image", cv2.WINDOW_NORMAL)
        # cv2.setMouseCallback("Live Image", self.mouse_callback)
        self.complete_define = False
        self.update_visualize = True

    def save_masked_gray_image(self, cv_image: np.ndarray, mask: np.ndarray, object_name: str):
        """
        Parameters:
        - cv_image: np.ndarray, BGR image
        - mask: np.ndarray, mask of bool
        - filename: str, filepath to save image (.png)
        """

        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # masked_gray = np.zeros_like(gray)
        # masked_gray[mask] = gray[mask]

        binary_mask = (mask.astype(np.uint8)) * 255
        bgr_path = os.path.join(self.package_dir, f"io/images/{object_name}.png")
        mask_path = os.path.join(self.package_dir, f"io/masks/{object_name}.png")
        cv2.imwrite(bgr_path, cv_image)
        # cv2.imwrite(mask_path, masked_gray)
        cv2.imwrite(mask_path, binary_mask)
        print(f"Masked grayscale image saved to: {bgr_path}")

    def define_callback(self):
        if self.complete_define is False:
            cv2.setMouseCallback("Live Image", self.mouse_callback)

    def mouse_callback(self, event, x, y, flags, param):
        if self.cv_image is None:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            # self.update_visualize = False
            self.click_start = (x, y)
            self.click_end = None
            self.clicked = False

        elif event == cv2.EVENT_LBUTTONUP:
            self.click_end = (x, y)
            self.clicked = True

            output_img = self.cv_image.copy()

            if self.click_start == self.click_end:
                # Single click → draw green dot
                cv2.circle(output_img, self.click_start, radius=5, color=(255, 0, 0), thickness=-1)
                print(f"Clicked at: {self.click_start}")
                use_bbox = False
            else:
                # Drag → draw green bbox
                x1, y1 = self.click_start
                x2, y2 = self.click_end
                top_left = (min(x1, x2), min(y1, y2))
                bottom_right = (max(x1, x2), max(y1, y2))
                cv2.rectangle(output_img, top_left, bottom_right, color=(0, 255, 0), thickness=2)
                print(f"BBOX from {top_left} to {bottom_right}")
                input_box = np.array([top_left[0], top_left[1], bottom_right[0], bottom_right[1]])
                use_bbox = True

            # Show the result in another window
            cv2.imshow("Clicked Region", output_img)
            cv2.waitKey(1)

            target_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(target_image)
            input_point = np.array([[x, y]])
            input_label = np.array([1])

            if use_bbox:
                masks, scores, logits = self.predictor.predict(
                    box=input_box[None, :],
                    point_labels=input_label,
                    multimask_output=True,
                )
            else:
                masks, scores, logits = self.predictor.predict(
                    point_coords=input_point,
                    point_labels=input_label,
                    multimask_output=True,
                )

            for i, (mask, score) in enumerate(zip(masks, scores)):
                # gray = cv2.cvtColor(target_image, cv2.COLOR_BGR2GRAY)
                # masked_gray = np.zeros_like(gray)
                # masked_gray[mask] = gray[mask]

                masked_image = np.zeros_like(target_image)
                masked_image[mask] = target_image[mask]
                masked_image = cv2.cvtColor(masked_image, cv2.COLOR_RGB2BGR)

                cv2.imshow(f"Mask {i}", masked_image)
                cv2.waitKey(10)
                print(f"Mask {i} score {score}", masked_image)

            self.update_visualize = True
            cv2.waitKey(10)

            mask_id = int(input("どのマスクを保存しますか？ 数字を入力してください．>>> "))
            object_name = input("物体名はなんですか？ 文字列を入力してください．>>> ")

            self.save_masked_gray_image(self.cv_image, masks[mask_id], object_name=object_name)

        self.update_visualize = True

    def image_callback(self, msg):
        try:
            np_arr: np.ndarray = np.frombuffer(msg.data, dtype=np.uint8)
            if self.update_visualize:
                self.cv_image: np.ndarray = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"cv_bridge exception: {e}")
            return

        if self.update_visualize:
            cv2.imshow("Live Image", self.cv_image)
            cv2.waitKey(1)
            self.define_callback()
            self.complete_define = True

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    cls = AutoMaskGenerator()
    cls.run()
