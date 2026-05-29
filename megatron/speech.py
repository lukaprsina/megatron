import logging

from std_msgs.msg import String


class Speaker:
    """Publishes text strings to the /speak topic for downstream TTS handling."""

    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._node_logger = None
        self._publisher = None

    def set_node_logger(self, node) -> None:
        """Use an rclpy Node for logging and create the /speak publisher.

        Example:
            speaker.set_node_logger(node)
        """
        try:
            self._node_logger = node.get_logger()
            self._logger = self._node_logger
            self._publisher = node.create_publisher(String, '/speak', 10)
            self._logger.debug('Speaker logger set to rclpy Node logger')
        except Exception:
            self._logger.warning('Failed to set node logger; continuing with std logger')

    def is_busy(self) -> bool:
        """Always returns False — publishing is instantaneous."""
        return False

    def speak(self, text: str) -> None:
        """Publish text to the /speak topic."""
        if self._publisher is None:
            self._logger.debug('speak() called but publisher is not available')
            return

        msg = String()
        msg.data = text
        self._publisher.publish(msg)
        self._logger.info(f"Published to /speak: {text[:100]}")
