import shutil
import subprocess


class Speaker:
    """Lightweight non-blocking TTS using espeak."""

    def __init__(self):
        self._espeak = shutil.which('espeak')
        self._process = None

    def speak(self, text: str) -> None:
        """Fire-and-forget speech synthesis. Non-blocking."""
        if self._espeak is None:
            return

        # Don't overlap with a currently speaking process
        if self._process is not None and self._process.poll() is None:
            return

        self._process = subprocess.Popen(
            [self._espeak, text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
