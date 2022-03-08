import wave
from pathlib import Path
import datetime


class AudioRecorder:
    def __init__(
        self,
        audio_rate=16000,
        audio_channels=1,
        audio_width=2,
        audio_type="wav",
        out_directory="audio",
    ):
        self._audio_rate = audio_rate
        self._audio_channels = audio_channels
        self._audio_width = audio_width
        self._audio_type = audio_type

        self._out_directory = out_directory

        self._is_recording = False

    def start_recording(self, out_file_name=None):

        if self._is_recording:
            raise RuntimeError("Audio is already being recorded")
        if not Path(self._out_directory).is_dir():
            Path(self._out_directory).mkdir()
        if out_file_name is None:
            ext = self._audio_type
            file_name = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            out_file_name = f"{file_name}.{ext}"

        out_file_path = Path(self._out_directory) / out_file_name
        out_file_path = str(out_file_path)
        self._wf = wave.open(out_file_path, "wb")
        self._wf.setnchannels(self._audio_channels)
        self._wf.setsampwidth(self._audio_width)
        self._wf.setframerate(self._audio_rate)
        self._is_recording = True

    def add_audio(self, audio, is_throw_error_if_not_recording=True):
        if self._is_recording:
            self._wf.writeframes(audio)
        else:
            if is_throw_error_if_not_recording:
                raise RuntimeError("Audio recording has not been started")

    def stop_recording(self):
        if not self._is_recording:
            raise RuntimeError("Audio recording was not started")

        self._wf.close()
        self._is_recording = False
