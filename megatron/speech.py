import shutil
import subprocess
import logging


class Speaker:
    """Lightweight non-blocking TTS using espeak-ng."""

    def __init__(self):
        self._espeak = shutil.which('espeak-ng')
        self._process = None
        # Logger similar to rclpy Node logging used in other modules
        self._logger = logging.getLogger(__name__)
        if self._espeak is None:
            self._logger.warning('espeak-ng not found on PATH; speech will be disabled')

            # If running inside an rclpy Node, you can call set_node_logger(node)
            # to forward logs to the node's logger (so messages appear with ROS2 formatting).
            self._node_logger = None

    def set_node_logger(self, node) -> None:
        """Use an rclpy Node's logger for output formatting and ROS2 integration.

        Example:
            speaker.set_node_logger(node)
        """
        try:
            self._node_logger = node.get_logger()
            # Replace internal logger methods with node logger methods
            self._logger = self._node_logger
            self._logger.debug('Speaker logger set to rclpy Node logger')
        except Exception:
            # If node doesn't provide get_logger, ignore and keep std logger
            self._logger.warning('Failed to set node logger; continuing with std logger')

    def speak(self, text: str) -> None:
        """Fire-and-forget speech synthesis. Non-blocking."""
        if self._espeak is None:
            self._logger.debug('speak() called but espeak-ng is not available')
            return

        # Don't overlap with a currently speaking process
        if self._process is not None and self._process.poll() is None:
            self._logger.debug('Previous espeak-ng process still running; skipping speak()')
            return

        try:
            self._process = subprocess.Popen(
                [self._espeak, text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._logger.info(f"Started espeak-ng process for text: {text[:100]}")
        except Exception as e:
            # Log unexpected errors when trying to start TTS
            self._logger.error(f"Failed to start espeak-ng: {e}", exc_info=True)
