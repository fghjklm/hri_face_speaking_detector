# hri_face_speaking_detector

## Overview

The `hri_face_speaking_detector` node is for face speaking detection using classification models. It integrates  within the ROS4HRI framework by analyzing the output from the `hri_face_detector` node to identify whether a face is speaking.

## Preparation

The face speaking detector node relies on pre-trained classifier.

## Resources

## ROS API

### Topics

- **`/humans/faces/[face_id]/is_speaking`**: This topic publishes a boolean corresponding to whether the face is speaking or not.
- `/diagnostics` ([diagnostic_msgs/DiagnosticArray](https://github.com/ros2/common_interfaces/blob/humble/diagnostic_msgs/msg/DiagnosticArray.msg))

## Parameters

- **`sequence_length`**: This parameter specifies the length of the image sequence the classifier uses. Note that changing that parameter woul requier to change the classifier, as it is trained with a fixed length.

## Dependencies

- This node requieres `numpy = 1.22.0` and `scikit-learn = 1.1.3`. 

## Launch

```bash
ros2 launch hri_face_speaking_detector face_speaking_detector.launch.py
```

The `face_speaking_detector.launch.py` launch file accepts as arguments and configures the defined [parameters](#parameters).
It also automatically transitions the node to the active state.

## Example

To test thepackage using the system default microphone:

1. Install the `usb_cam` package:
   `sudo apt install ros-humble-usb-cam`
1. Launch the `usb_cam` package:
   `ros2 run usb_cam usb_cam_node_exe`
1. In a new terminal, install launch the `hri_face_detect` package:
   `sudo apt install ros-humble-hri-face-detect`
   `ros2 launch hri_face_detect face_detect.launch.py rgb_camera:=<input camera namespace>`
1. In a new terminal, run the `hri_emotion_recognizer` package: 
   `ros2 launch hri_face_speaking_detector face_speaking_detector.launch.py`
1. Check the faces tracked and the corresponding expression
   `ros2 topic echo /humans/faces/tracked`
   `ros2 topic echo /humans/faces/wstw/is_speaking`




