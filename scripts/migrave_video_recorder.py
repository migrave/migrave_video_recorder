#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import cv2

from video_recorder.video_recorder import video_recorder


class VideoCapture:
    def __init__(
        self,
        is_record_topic,
        color_image_topic,
        depth_image_topic,
        video_type,
        video_dimensions_color,
        video_dimensions_depth,
        frames_per_second,
        out_directory,
    ):

        rospy.init_node("migrave_video_recorder", anonymous=True)
        self._color_video_recorder = video_recorder(
            video_type=video_type,
            video_dimensions=video_dimensions_color,
            frames_per_second=frames_per_second,
            out_directory=out_directory,
        )

        self._depth_video_recorder = video_recorder(
            video_type=video_type,
            video_dimensions=video_dimensions_depth,
            frames_per_second=frames_per_second,
            out_directory=out_directory,
        )

        self._is_record_subscriber = rospy.Subscriber(
            is_record_topic, Bool, self._is_record_callback
        )
        self._color_image_subscriber = rospy.Subscriber(
            color_image_topic, Image, self._color_image_callback
        )
        self._depth_image_subscriber = rospy.Subscriber(
            depth_image_topic, Image, self._depth_image_callback
        )
        self._bridge = CvBridge()

        # This flag is used to block recording if memory exceeeds limits
        self._allow_recording = True  # TODO add a memory usage watch topic

    def _color_image_callback(self, data):

        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            raise e

        self._color_video_recorder.add_image(
            cv_image, is_throw_error_if_not_recording=False
        )

    def _depth_image_callback(self, data):

        try:
            cv_image = self._bridge.imgmsg_to_cv2(
                data, data.encoding)  # z16 format
            cv_image = cv2.applyColorMap(
                cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET
            )
            # cv_image = self._bridge.imgmsg_to_cv2(
            #     data, desired_encoding="16UC1")  # z16 format
        except CvBridgeError as e:
            raise e

        self._depth_video_recorder.add_image(
            cv_image, is_throw_error_if_not_recording=False
        )

    def _is_record_callback(self, data):

        is_record = data.data
        try:
            if is_record:
                if self._allow_recording:
                    # rospy.loginfo("Starting to record video")
                    ext = video_type
                    file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    color_video_file = f"color_{file_name}.{ext}"
                    depth_video_file = f"depth_{file_name}.{ext}"
                    rospy.loginfo("Starting to record color video")
                    self._color_video_recorder.start_recording(
                        out_file_name=color_video_file
                    )
                    rospy.loginfo("Starting to record depth video")
                    self._depth_video_recorder.start_recording(
                        out_file_name=depth_video_file
                    )
                else:
                    rospy.logerr(
                        "Recording will not happen " "due to memory limits exceeded"
                    )
            else:
                if self._color_video_recorder._is_recording:
                    rospy.loginfo("Stopped recording color video")
                    self._color_video_recorder.stop_recording()
                if self._depth_video_recorder._is_recording:
                    rospy.loginfo("Stopped recording depth video")
                    self._depth_video_recorder.stop_recording()

        except RuntimeError as e:
            rospy.logerr(e)


if __name__ == "__main__":
    # default parameters
    color_image_topic = "/camera/color/image_raw"
    # depth_image_topic = "/camera/depth/image_raw"
    depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
    is_record_topic = "/migrave_data_recording/is_record"
    video_type = "mp4"
    video_dimensions_color = "480p"
    video_dimensions_depth = "480p"
    frames_per_second = 30
    output_directory = "/home/qtrobot/Documents"
    # update parameter if provided by ros parameter server
    parameter_name = "/migrave_video_recorder/color_image_topic"
    if rospy.has_param(parameter_name):
        color_image_topic = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/depth_image_topic"
    if rospy.has_param(parameter_name):
        depth_image_topic = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/is_record_topic"
    if rospy.has_param(parameter_name):
        is_record_topic = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/is_record_topic"
    if rospy.has_param(parameter_name):
        is_record_topic = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/video_type"
    if rospy.has_param(parameter_name):
        video_type = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/video_dimensions_color"
    if rospy.has_param(parameter_name):
        video_dimensions_color = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/video_dimensions_depth"
    if rospy.has_param(parameter_name):
        video_dimensions_depth = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/frames_per_second"
    if rospy.has_param(parameter_name):
        frames_per_second = rospy.get_param(parameter_name)

    parameter_name = "/migrave_video_recorder/out_directory"
    if rospy.has_param(parameter_name):
        out_directory = rospy.get_param(parameter_name)

    VideoCapture(
        color_image_topic=color_image_topic,
        depth_image_topic=depth_image_topic,
        is_record_topic=is_record_topic,
        video_type=video_type,
        video_dimensions_color=video_dimensions_color,
        video_dimensions_depth=video_dimensions_depth,
        frames_per_second=frames_per_second,
        out_directory=output_directory,
    )

    rospy.spin()
