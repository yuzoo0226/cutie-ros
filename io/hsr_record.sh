#!/bin/bash

# 第一引数をファイル名として取得（なければ空）
BAG_NAME=$1

# トピックリスト
# TOPICS=(
#   /hsrb/hand_camera/camera_info
#   /hsrb/hand_camera/hand_camera_nodelet_manager/bond
#   /hsrb/hand_camera/image_raw
#   /hsrb/hand_camera/image_raw/compressed
#   /hsrb/hand_camera/image_raw/compressedDepth
#   /hsrb/head_rgbd_sensor/depth_registered/camera_info
#   /hsrb/head_rgbd_sensor/depth_registered/image
#   /hsrb/head_rgbd_sensor/depth_registered/image/compressed
#   /hsrb/head_rgbd_sensor/depth_registered/image/compressedDepth
#   /hsrb/head_rgbd_sensor/rgb/image_color
#   /hsrb/head_rgbd_sensor/rgb/image_color/compressed
#   /hsrb/head_rgbd_sensor/rgb/image_color/compressedDepth
#   /hsrb/head_rgbd_sensor/rgb/image_raw
#   /hsrb/head_rgbd_sensor/rgb/image_raw/compressed
#   /hsrb/head_rgbd_sensor/rgb/image_raw/compressedDepth
#   /hsrb/head_rgbd_sensor/rgb/image_raw/compressedDepth/parameter_descriptions
# )

TOPICS=(
  /hsrb/hand_camera/camera_info
  # /hsrb/hand_camera/hand_camera_nodelet_manager/bond
  # /hsrb/hand_camera/image_raw
  /hsrb/hand_camera/image_raw/compressed
  # /hsrb/hand_camera/image_raw/compressedDepth
  /hsrb/head_rgbd_sensor/depth_registered/camera_info
  # /hsrb/head_rgbd_sensor/depth_registered/image
  # /hsrb/head_rgbd_sensor/depth_registered/image/compressed
  /hsrb/head_rgbd_sensor/depth_registered/image/compressedDepth
  /hsrb/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth
  # /hsrb/head_rgbd_sensor/rgb/image_color
  /hsrb/head_rgbd_sensor/rgb/image_color/compressed
  # /hsrb/head_rgbd_sensor/rgb/image_color/compressedDepth
  # /hsrb/head_rgbd_sensor/rgb/image_raw
  /hsrb/head_rgbd_sensor/rgb/image_raw/compressed
  # /hsrb/head_rgbd_sensor/rgb/image_raw/compressedDepth
  # /hsrb/head_rgbd_sensor/rgb/image_raw/compressedDepth/parameter_descriptions
)

# rosbag record コマンドの構築
if [ -n "$BAG_NAME" ]; then
  rosbag record -o "$BAG_NAME" "${TOPICS[@]}"
else
  rosbag record "${TOPICS[@]}"
fi
