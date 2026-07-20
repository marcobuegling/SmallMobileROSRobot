import time
import os
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder, JpegEncoder
from picamera2.outputs import FileOutput, CircularOutput


class PiCamera:
    """
    Interface for managing a camera using the picamera2 module.
    Supports Pi Camera Modules and compatible cameras using the Pi's camera port.

    Modes:
        image  - capture stills as JPG (single shot or series)
        video  - record H.264 video to a file
        stream - yield individual frames for live processing
    """

    VALID_MODES = ('image', 'video', 'stream')

    def __init__(self, mode: str = 'image', path: str = '', buffer_size: int = 0):
        self._cam = Picamera2()
        self._h264_encoder = H264Encoder(bitrate=10_000_000)
        self._jpeg_encoder = JpegEncoder()

        self._path = path
        self._buffer_size = buffer_size
        self._mode = None  # will be set by change_camera_mode below

        self.change_camera_mode(mode)

        if buffer_size > 0:
            # CircularOutput keeps the last `buffer_size` seconds of encoded data
            # in memory - useful for "save the last N seconds" workflows.
            self._circular_output = CircularOutput(buffersize=buffer_size)

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def change_camera_mode(self, mode: str) -> None:
        """Change the operating mode of the camera. Restarts the camera if running."""
        if mode not in self.VALID_MODES:
            raise ValueError(f"Invalid mode '{mode}'. Choose from: {self.VALID_MODES}")

        # Stop the camera cleanly before reconfiguring
        try:
            self._cam.stop()
        except Exception:
            pass  # not running yet

        if mode == 'image':
            self._cam.configure(self._cam.create_still_configuration())
        elif mode == 'video':
            self._cam.configure(self._cam.create_video_configuration())
        elif mode == 'stream':
            # Preview configuration gives low-latency frames suitable for real-time processing without writing to disk.
            self._cam.configure(self._cam.create_preview_configuration())

        self._mode = mode

    # ------------------------------------------------------------------
    # Image capture
    # ------------------------------------------------------------------

    def single_image(self, output_path: str = None) -> str:
        """
        Capture a single still image.

        Args:
            output_path: Destination file path. Falls back to self._path.

        Returns:
            The path the image was saved to.

        Raises:
            RuntimeError: If the camera is not in 'image' mode.
        """
        self._require_mode('image')
        path = self._resolve_path(output_path, suffix='.jpg')
        self._ensure_dir(path)

        self._cam.start()
        self._cam.capture_file(path)
        self._cam.stop()

        return path

    def image_series(self, n: int, interval: float = 0.1, output_path: str = None) -> list[str]:
        """
        Capture a series of n still images spaced `interval` seconds apart.

        Args:
            n:           Number of images to capture.
            interval:    Seconds between captures.
            output_path: Directory to save images in. Falls back to self._path.

        Returns:
            List of file paths that were written.

        Raises:
            RuntimeError: If the camera is not in 'image' mode.
            ValueError:   If n < 1 or interval < 0.
        """
        self._require_mode('image')
        if n < 1:
            raise ValueError("n must be at least 1")
        if interval < 0:
            raise ValueError("interval must be non-negative")

        base_dir = output_path or self._path or '.'
        self._ensure_dir(base_dir + '/')
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        paths = []

        self._cam.start()
        for i in range(n):
            file_path = os.path.join(base_dir, f"{timestamp}_{i:04d}.jpg")
            self._cam.capture_file(file_path)
            paths.append(file_path)
            if i < n - 1:
                time.sleep(interval)
        self._cam.stop()

        return paths

    # ------------------------------------------------------------------
    # Video recording
    # ------------------------------------------------------------------

    def single_video(self, duration: float, output_path: str = None) -> str:
        """
        Record a video of the given duration and save it as H.264.

        Args:
            duration:    Recording length in seconds.
            output_path: Destination file path. Falls back to self._path.

        Returns:
            The path the video was saved to.

        Raises:
            RuntimeError: If the camera is not in 'video' mode.
            ValueError:   If duration <= 0.
        """
        self._require_mode('video')
        if duration <= 0:
            raise ValueError("duration must be positive")

        path = self._resolve_path(output_path, suffix='.h264')
        self._ensure_dir(path)

        self._cam.start()
        self._cam.start_encoder(self._h264_encoder, FileOutput(path))
        time.sleep(duration)
        self._cam.stop_encoder()
        self._cam.stop()

        return path

    # ------------------------------------------------------------------
    # Streaming / live frame access
    # ------------------------------------------------------------------

    def video_stream_frame(self) -> np.ndarray:
        """
        Capture and return a single frame as a NumPy array (RGB).

        Intended for live processing pipelines (e.g. object tracking).
        The camera is started on the first call and left running for
        efficiency — call close() when done.

        Returns:
            numpy.ndarray: shape (height, width, 3), dtype uint8.

        Raises:
            RuntimeError: If the camera is not in 'stream' mode.
        """
        self._require_mode('stream')

        if not self._cam.started:
            self._cam.start()

        return self._cam.capture_array('main')

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Stop the camera and release hardware resources."""
        try:
            self._cam.stop()
        except Exception:
            pass
        self._cam.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_mode(self, expected: str) -> None:
        if self._mode != expected:
            raise RuntimeError(
                f"Camera must be in '{expected}' mode for this operation "
                f"(currently in '{self._mode}' mode)."
            )

    def _resolve_path(self, override: str | None, suffix: str) -> str:
        """Return override if given, otherwise build a timestamped path from self._path."""
        if override:
            return override
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        base = self._path or '.'
        return os.path.join(base, f"capture_{timestamp}{suffix}")

    @staticmethod
    def _ensure_dir(path: str) -> None:
        """Create parent directories for `path` if they don't exist."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
