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

- - When running in apptainer, build ros_ws under `--fakeroot --writable` and make sure the node is started with `rosrun cutie_ros cutie_service_node.py`.

```bash
cd src
python cutie/utils/download_models.py
```

## How to use

```bash
rosrun cutie_ros cutie_service_node.py
```
