import abc
from typing import Callable, Any

class BaseTransport(abc.ABC):
    def __init__(self, config: dict):
        self.config = config
        self.downlink_callback = None

    def set_downlink_callback(self, callback: Callable[[dict], None]):
        """
        Set the callback function to be called when a downlink command is received.
        The callback should accept a dictionary containing command_type and params.
        """
        self.downlink_callback = callback

    @abc.abstractmethod
    def connect(self) -> bool:
        pass

    @abc.abstractmethod
    def disconnect(self):
        pass

    @abc.abstractmethod
    def send_telemetry(self, msg: Any):
        """Send Telemetry.msg"""
        pass

    @abc.abstractmethod
    def send_health(self, msg: Any):
        """Send SystemHealth.msg"""
        pass
    
    @abc.abstractmethod
    def send_robot_state(self, msg: Any):
        """Send RobotState.msg"""
        pass

    @abc.abstractmethod
    def send_ack(self, sequence_id: int, success: bool, message: str = ""):
        """Send ACK for a received DownlinkCommand"""
        pass
