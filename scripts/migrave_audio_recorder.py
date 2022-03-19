#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import Bool
from datetime import datetime
import os

from audio_recorder.audio_recorder import AudioRecorder


class AudioCapture:
    def __init__(
        self,
        is_record_topic,
        audio_topic,
        audio_rate,
        audio_channels,
        audio_width,
        audio_type,
        out_directory,
    ):

        self._audio_recorder = AudioRecorder(
            audio_rate=audio_rate,
            audio_channels=audio_channels,
            audio_width=audio_width,
            audio_type=audio_type,
            out_directory=out_directory,
        )

        self._is_record_subscriber = rospy.Subscriber(
            is_record_topic, Bool, self._is_record_callback
        )
        self._audio_subscriber = rospy.Subscriber(
            audio_topic, AudioData, self._audio_callback
        )

        # This flag is used to block recording if memory exceeeds limits
        self._allow_recording = True

    def _audio_callback(self, data):

        self._audio_recorder.add_audio(
            data.data, is_throw_error_if_not_recording=False
        )

    def _is_record_callback(self, data):

        is_record = data.data
        try:
            if is_record:
                if self._allow_recording:
                    rospy.loginfo("Starting to record audio")
                    ext = self._audio_recorder._audio_type
                    file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    audio_file = f"{file_name}.{ext}"
                    self._audio_recorder.start_recording(
                        out_file_name=audio_file
                    )
                else:
                    rospy.logerr(
                        "Recording will not happen "
                        "due to memory limits exceeded"
                    )
            else:
                if self._audio_recorder._is_recording:
                    rospy.loginfo("Stopped recording audio")
                    self._audio_recorder.stop_recording()

        except RuntimeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node("migrave_audio_recorder", anonymous=True)
    node_name = rospy.get_name()
    path = rospy.get_param(node_name+'/output_directory', "/home/qtrobot/1/audio/")
    audio_topic = rospy.get_param(node_name+'/channel_topic', "/qt_respeaker_app/channel0")
    is_record_topic = rospy.get_param(node_name+'/is_record_topic', "/qt_robot_audio_recording/is_record")
    audio_rate = rospy.get_param(node_name+'/audio_rate', 16000)
    audio_channels = rospy.get_param(node_name+'/audio_channel', 1)
    audio_width = rospy.get_param(node_name+'/audio_width', 2)
    audio_type = rospy.get_param(node_name+'audio_type', "wav")

    try:
        os.makedirs(path)
    except FileExistsError as err:
        print(f"WARNING: Directory {path} already exists")

    AudioCapture(
        is_record_topic=is_record_topic,
        audio_topic=audio_topic,
        audio_rate=audio_rate,
        audio_channels=audio_channels,
        audio_width=audio_width,
        audio_type=audio_type,
        out_directory=path,
    )

    rospy.spin()
