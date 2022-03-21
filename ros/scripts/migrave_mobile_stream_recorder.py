#!/usr/bin/env python3

import cv2
import rospy
import time
from pathlib import Path
from std_msgs.msg import Bool

class MobileStreamRecorder:
    video_type_code = {
        "avi": cv2.VideoWriter_fourcc(*"DIVX"),
        "mp4": cv2.VideoWriter_fourcc(*"mp4v"),
    }

    def __init__(
        self,
        stream_url,
        is_record_topic,
        video_type="mp4",
        frames_per_second=30,
        out_directory="video",
    ):
        if video_type not in self.video_type_code:
            raise KeyError(
                    f"Invalid video type, "
                    f"please use one of the following:"
                    f"{self.video_type_code.keys()}"
            )

        self._video_type = video_type
        self._video_dimensions = (540, 1156)
        self._frames_per_second = frames_per_second

        self._out_directory = out_directory

        self._is_recording = False
        self._video_writer = None
        self._timestamp_writer = None
        self._allow_recording = True

        self._is_record_subscriber = rospy.Subscriber(
            is_record_topic, Bool, self._is_record_callback
        )
        self._timestamp_writer = None
        self._video_writer = None
        self._cap = cv2.VideoCapture(stream_url)

        while True:
            is_opened = self._cap.isOpened()
            if not is_opened:
                rospy.logwarn("Could not intialize video capture, retrying ...")
                self._cap.open(stream_url)
                rospy.sleep(1)
            else:
                break

    def __del__(self):
        if self._timestamp_writer is not None:
            self._timestamp_writer.close()

    def _is_record_callback(self, data):
        is_record = data.data
        try:
            if is_record:
                if not self._is_recording:
                    self.recording_setup()
                    rospy.loginfo("Starting to record mobile stream")
                    self._is_recording = True
                else:
                    rospy.logwarn("Mobile stream is already being recorded")
            else:
                if self._is_recording:
                    rospy.loginfo("Stopped to record mobile stream")
                    self._is_recording = False
                else:
                    rospy.logwarn("Mobile stream is already stopped")

        except RuntimeError as e:
            rospy.logerr(e)

    def recording_setup(self, out_file_name=None):
        if not Path(self._out_directory).is_dir():
            Path(self._out_directory).mkdir()

        if out_file_name is None:
            ext = self._video_type
            t = time.localtime()
            file_name = time.strftime("%Y-%m-%d_%H-%M-%S", t)
            out_file_name = f"mobile_stream_{file_name}.{ext}"

        out_file_path = Path(self._out_directory) / out_file_name
        out_file_path = str(out_file_path)

        self._video_writer = cv2.VideoWriter(
            out_file_path,
            self.video_type_code[self._video_type],
            self._frames_per_second,
            self._video_dimensions,
            True,
        )
        timestamp_file_name = out_file_path[:-3] + 'txt'
        self._timestamp_writer = open(timestamp_file_name, "a")

    def run_recording(self):
        rate = rospy.Rate(self._frames_per_second)

        while not rospy.is_shutdown():
            if self._is_recording:
                ret, frame = self._cap.read()
                self.add_image(frame)
            else:
                self.stop_recording()
                rospy.sleep(2)
            rate.sleep()

    def add_image(self, image, is_throw_error_if_not_recording=True):
        if self._is_recording:
            timestamp = str(int(round(time.time() * 1000))) + "\n"
            self._timestamp_writer.write(timestamp)
            self._video_writer.write(image)

    def stop_recording(self):
        self._video_writer = None
        self._is_recording = False

        if self._timestamp_writer is not None:
            self._timestamp_writer.close()


if __name__ == "__main__":
    rospy.init_node("migrave_mobile_stream_recorder", anonymous=True)
    node_name = rospy.get_name()
    path = rospy.get_param(node_name+'/output_directory')
    is_record_topic = rospy.get_param(node_name+'/is_record_topic', "/qt_robot_video_recording/is_record")
    video_type = rospy.get_param(node_name+'/video_type', "mp4")
    frames_per_second = rospy.get_param(node_name+'/frames_per_second', 15)
    stream_url = rospy.get_param(node_name+'/stream_url', 'http://localhost:8080/stream.mjpeg')

    stream_recorder = MobileStreamRecorder(stream_url=stream_url,
                                           video_type=video_type,
                                           frames_per_second=frames_per_second,
                                           out_directory=path,
                                           is_record_topic=is_record_topic)
    try:
        stream_recorder.run_recording()
    except (KeyboardInterrupt, SystemExit):
        print('{0} interrupted; exiting...')
    
