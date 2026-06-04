import logging
import queue
import shutil
import subprocess
import threading
from datetime import datetime


class Speaker:
    """Non-blocking TTS using espeak-ng with a queue so no lines are dropped.

    Singleton — all nodes share one queue, one espeak-ng process, one log file.
    """

    _instance: "Speaker | None" = None
    _lock = threading.Lock()

    def __new__(cls) -> "Speaker":
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, "_initialized"):
            return
        self._initialized = True
        self._espeak = shutil.which("espeak-ng")
        self._logger = logging.getLogger(__name__)
        self._node_logger = None
        self._logfile = open("speech.log", "w", encoding="utf-8")
        self._queue: queue.SimpleQueue[str] = queue.SimpleQueue()
        self._process: subprocess.Popen | None = None
        self._running = True
        self._log("--- Speaker initialized ---")
        if self._espeak is None:
            self._logger.warning("espeak-ng not found on PATH; speech will be disabled")
            self._log("espeak-ng not found on PATH; speech disabled")
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def _log(self, msg: str) -> None:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"[{ts}] {msg}"
        try:
            self._logfile.write(line + "\n")
            self._logfile.flush()
        except Exception:
            pass

    def set_node_logger(self, node) -> None:
        """Use an rclpy Node's logger for output formatting and ROS2 integration.

        Example:
            speaker.set_node_logger(node)
        """
        try:
            self._node_logger = node.get_logger()
            self._logger = self._node_logger
            self._logger.debug("Speaker logger set to rclpy Node logger")
            self._log("Logger set to rclpy Node logger")
        except Exception:
            self._logger.warning(
                "Failed to set node logger; continuing with std logger"
            )

    def is_busy(self) -> bool:
        """Return True if the queue is non-empty or espeak-ng is currently speaking."""
        if not self._queue.empty():
            return True
        return self._process is not None and self._process.poll() is None

    def speak(self, text: str) -> None:
        """Enqueue text for speech synthesis. Non-blocking, never drops."""
        if self._espeak is None:
            return
        self._queue.put(text)

    def shutdown(self) -> None:
        """Stop the worker thread and clean up."""
        self._running = False
        self._queue.put("")
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            self._process.wait()
        self._log("--- Speaker shutdown ---")

    def _run(self) -> None:
        while self._running:
            try:
                text = self._queue.get()
            except Exception:
                continue
            if not text or text == "":
                continue
            espeak = self._espeak
            if espeak is None:
                continue
            self._logger.info(f"Speaking: {text[:100]}")
            try:
                self._process = subprocess.Popen(
                    [espeak, "-s", "150", text],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self._process.wait()
                self._log(f"DONE: {text[:200]}")
            except Exception as e:
                self._logger.error(f"Failed to start espeak-ng: {e}")
                self._log(f"ERROR: {e} (text: {text[:200]})")
