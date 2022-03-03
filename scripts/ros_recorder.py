#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import sys, getopt
import os
from video_recorder.video_recorder import VideoRecorder


class VideoCapture:
    def __init__(
        self,
        is_record_topic,
        color_image_topic,
        depth_image_topic,
        video_type,
        video_dimensions,
        frames_per_second,
        out_directory,
    ):

        rospy.init_node("video_ros_recorder", anonymous=True)
        self._color_video_recorder = VideoRecorder(
            video_type=video_type,
            video_dimensions=video_dimensions,
            frames_per_second=frames_per_second,
            out_directory=out_directory,
        )

        self._depth_video_recorder = VideoRecorder(
            video_type=video_type,
            video_dimensions=video_dimensions,
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
            cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
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
                        "Recording will not happen "
                        "due to memory limits exceeded"
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

def main(argv):
    user = ''
    participant = ''
    try:
        opts, args = getopt.getopt(argv,"hu:p:",["user=","participant="])
    except getopt.GetoptError:
        print('test.py -u <user> -p <participant>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('test.py -u <user> -p <participant>')
            sys.exit()
        elif opt in ("-u", "--user"):
            user = arg
        elif opt in ("-p", "--participant"):
            participant = arg

    print(f"PC user is {user}, participant is {participant}")
    return user, participant

if __name__ == "__main__":
    user = 'michal'
    participant = '1'
    #user, participant = main(sys.argv[1:])
    path = f"/home/{user}/videos/{participant}"
    try:
        os.makedirs(path)
    except FileExistsError as err:
        print(f"WARNING: Directory {path} already exists")

    output_directory = rospy.get_param('~output_directory', path)
    color_image_topic = rospy.get_param('~color_image_topic', '/camera/color/image_raw')
    depth_image_topic = rospy.get_param('~depth_image_topic', '/camera/depth/image_raw')
    is_record_topic = rospy.get_param('~is_record_topic', "/qt_robot_video_recording/is_record")
    video_type = rospy.get_param('~video_type', "mp4")
    video_dimensions = rospy.get_param('~video_dimensions', "480p")
    frames_per_second = rospy.get_param('~frames_per_second', 30)

    VideoCapture(
        color_image_topic=color_image_topic,
        depth_image_topic=depth_image_topic,
        is_record_topic=is_record_topic,
        video_type=video_type,
        video_dimensions=video_dimensions,
        frames_per_second=frames_per_second,
        out_directory=output_directory,
    )

    rospy.spin()
