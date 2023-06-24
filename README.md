# HBV-1780 ROS Wrapper

## Test run
You can run __single camera__ case by (we assume the camera is mounted at `/dev/video2`):
```
roslaunch hbv_1780_ros hbv_1780.launch
```

For __two camera__ case:
```
roslaunch hbv_1780_ros hbv_1780_two_cameras.launch
```

## Advanced
Please note that this node has three parameters:
* /<node_name>/left_camera_config_file (string): path to the calibration parameters of the left camera
* /<node_name>/right_camera_config_file (string): path to the calibration parameters of the right camera4
* /<node_name>/device (string): camera device, such as /dev/video2, which is the default value