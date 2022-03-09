# migrave_video_recorder
Repository for a video recorder used in the MigrAVE project

The package contains two nodes:
- migrave_video_recorder 
    - Video recorder for QTrobot's internal [Intel RealSense D455 camera](https://www.intelrealsense.com/depth-camera-d455) 
- migrave_webcam_recorder
    - Video recorder for the two external Logitech C930e usb webcams.

## Dependencies

  - std_msgs
  - cv_bridge
  - usb_cam
  - python-opencv
  - realsense2-camera

## Usage 

Launch the two recorder nodes:
```sh
roslaunch migrave_video_recorder migrave_video_recorder.launch
```
Launcn the ROS driver for the two webcams (if they are connected):
```sh
roslaunch migrave_video_recorder usb_cam_both.launch
```
_Note: the two cameras are assumed to be mounted as `/dev/video6` and `/dev/video8`, respectively._

Launch the official Intel ROS driver `realsense2-camera` for the RealSense camera (remember to disable `qt_nuitrack_app` first):
```sh
roslaunch migrave_video_recorder rs_camera.launch
```
_Note: it is recommended to use the Intel ROS driver `realsense2-camera` instead of the built-in `qt_nuitrack_app`. Because the Intel ROS driver is able to publish messages at the relevant image topics at a fixed frequency (30 Hz, here). While, the `qt_nuitrack_app` publishes messages at the image topics at a varying frequency (22-28 Hz) as it is using a `roscpp`'s timer which is not hard realtime timer. The video recorder requires image messages published at a fixed frequency, otherwise the video length will be different from the actual time length._

Start recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "True"
```
Stop recording

```sh
rostopic pub /migrave_data_recorder/is_record std_msgs/Bool "False"
```
