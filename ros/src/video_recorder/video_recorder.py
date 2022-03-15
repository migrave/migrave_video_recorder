import cv2
import datetime
from pathlib import Path
import rospy

class VideoRecorder:
    video_type_code = {
        "avi": cv2.VideoWriter_fourcc(*"DIVX"),
        "mp4": cv2.VideoWriter_fourcc(*"mp4v"),
    }

    std_dimensions = {
        "480p": (640, 480),
        "480p_d": (848, 480),
        "720p": (1280, 720),
        "1080p": (1920, 1080),
        "4k": (3840, 2160),
    }

    def __init__(
        self,
        video_type="mp4",
        video_dimensions="720p",
        frames_per_second=30,
        out_directory="video",
    ):
        if video_type not in self.video_type_code:
            raise KeyError(
                    f"Invalid video type, "
                    f"please use one of the following:"
                    f"{self.video_type_code.keys()}"
            )

        if video_dimensions not in self.std_dimensions:
            raise KeyError(
                    f"Invalid video dimensions, "
                    f"please use one of the following:"
                    f"{self.std_dimensions.keys()}"
            )

        self._video_type = video_type
        self._video_dimensions = self.std_dimensions[video_dimensions]
        self._frames_per_second = frames_per_second

        self._out_directory = out_directory

        self._is_recording = False
        self._video_writer = None

    def start_recording(self, out_file_name=None):

        if self._is_recording:
            raise RuntimeError("Video is already being recorded")

        # parameter set by migrave_games
        parameter_name = "/migrave/game_performance/participant_id"
        if rospy.has_param(parameter_name):
            participant_id = rospy.get_param(parameter_name)
            if participant_id == "":
                rospy.loginfo("Participant ID not set!")
                participant_id = "ID_Unknown"
        else:
            participant_id = "ID_Unknown"

        self._out_directory_id = Path(self._out_directory) / participant_id
        if not Path(self._out_directory_id).is_dir():
            Path(self._out_directory_id).mkdir(parents=True, exist_ok=True)
        if out_file_name is None:
            ext = self._video_type
            now = datetime.datetime.now()
            # time in YYYY-MM-DD_HH_MM-SS format
            time = now.strftime("%Y-%m-%d_%H-%M-%S")
            # unixtimestamp 16 digits
            stamp = int(datetime.datetime.timestamp(now) * 1000000)
            file_name = f"{time}_{stamp}"
            out_file_name = f"{file_name}.{ext}"

        out_file_path = Path(self._out_directory_id) / out_file_name
        out_file_path = str(out_file_path)
        self._video_writer = cv2.VideoWriter(
            out_file_path,
            self.video_type_code[self._video_type],
            self._frames_per_second,
            self._video_dimensions,
            True,
        )
        self._is_recording = True

    def add_image(self, image, is_throw_error_if_not_recording=True):

        if self._is_recording:
            self._video_writer.write(image)
        else:
            if is_throw_error_if_not_recording:
                raise RuntimeError("Video recording has not been started")

    def stop_recording(self):

        if not self._is_recording:
            raise RuntimeError("Video recording was not started")

        self._video_writer = None
        self._is_recording = False
