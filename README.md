# CUTIE for ros Noetic

## How to install

### Environment

- prease refer to the [apptainer definition file](https://github.com/yuzoo0226/singularity_definition_zoo/tree/main/024_object_tracking/cutie_noetic).

### Clone

```bash
git clone https://github.com/yuzoo0226/cutie-ros.git
git clone https://github.com/tamhome/tam_object_detection.git --recursive-submodule
```

## Download weight(s)

- When running in apptainer, build ros_ws under `--fakeroot --writable` and make sure the node is started with `rosrun cutie_ros cutie_service_node.py`.

```bash
cd scripts/
python download_weights_fakeroot.py
```

```bash
cd src
python cutie/utils/download_models.py
```

## How to use

### launch service node

```bash
rosrun cutie_ros cutie_service_node.py
```

### service request

```python
import rospy
from cutie_ros.srv import StartTracking, StartTrackingRequest

def temp():
    mask_msgs = []
    image_msgs = []

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

```