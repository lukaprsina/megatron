"""Lightweight non-blocking TTS wrapper around espeak-ng."""

import shutil
import subprocess
import logging


class Speaker:
    """Fire-and-forget speech synthesis using espeak-ng.

    Prevents overlapping speech and exposes ``is_busy()`` for queue-based
    callers in the mission controller.
    """

    def __init__(self) -> None:
        self._espeak = shutil.which('espeak-ng')
        self._process = None
        self._logger = logging.getLogger(__name__)
        self._node_logger = None
        if self._espeak is None:
            self._logger.warning(
                'espeak-ng not found on PATH; speech will be disabled')

    def set_node_logger(self, node) -> None:
        """Forward logs to an rclpy Node logger."""
        try:
            self._node_logger = node.get_logger()
            self._logger = self._node_logger
        except Exception:
            pass

    def is_busy(self) -> bool:
        """Return True if a speech process is currently running."""
        if self._process is None:
            return False
        return self._process.poll() is None

    def speak(self, text: str) -> None:
        """Start espeak-ng in a non-blocking subprocess.

        Skips if a previous utterance is still playing.
        """
        if self._espeak is None:
            self._logger.debug('speak() called but espeak-ng is not available')
            return

        if self.is_busy():
            self._logger.debug('espeak-ng still running; skipping')
            return

        try:
            self._process = subprocess.Popen(
                [self._espeak, text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._logger.info(f'Speaking: "{text[:80]}"')
        except Exception as e:
            self._logger.error(f'Failed to start espeak-ng: {e}')
