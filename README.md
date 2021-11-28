# migrave_video_recorder
Repository for a video recorder used in the MigrAVE project

# Usage
Start

```sh
rostopic pub /qt_robot_video_recording/is_record std_msgs/Bool "True"
```

Stop

```sh
rostopic pub /qt_robot_video_recording/is_record std_msgs/Bool "False"
```
