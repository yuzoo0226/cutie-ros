#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageViewerNode:
    def __init__(self):
        rospy.init_node("image_viewer_node", anonymous=True)
        self.bridge = CvBridge()

        # 表示する画像トピックをパラメータから取得（デフォルトあり）
        topic_name = rospy.get_param("~image_topic", "/object_detection/image")

        rospy.Subscriber(topic_name, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to image topic: {topic_name}")

    def image_callback(self, msg: Image):
        try:
            # ROS画像メッセージをOpenCV画像(BGR)に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr(f"cv_bridge exception: {e}")
            return

        # OpenCVで表示
        cv2.imshow("Live Image", cv_image)
        cv2.waitKey(1)

    def spin(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = ImageViewerNode()
    node.spin()
