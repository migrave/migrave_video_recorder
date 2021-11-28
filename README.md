# migrave_video_recorder
Repository for a video recorder used in the MigrAVE project

The node will start recording when publishing a `True` message (`std_msgs/Bool`) to the topic `/qt_robot_video_recording/is_record`, and stop with a `False` message.
