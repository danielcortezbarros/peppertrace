from enum import Enum


class EventType(Enum):
    """Generic event types."""

    DATA_LOGGING = 0
    ROBOT_CONTROL = 1
    TERMINATE = 2


class Event:
    """A generic event class that can encapsulate any type of event."""

    def __init__(self, event_type: EventType, command=None, info=""):
        """
        Creates an event with specified information.
        :param event_type: Type of the event (DATA_LOGGING or ROBOT_CONTROL)
        :param command: Event
        :param info: Information string
        """
        self.event_type = event_type
        self.command = command
        self.info = info


class DataLoggerCommands(Enum):
    START = 0
    STOP = 1
    PAUSE = 2
    SAVE = 3
    SAVE_SNAPSHOT = 4
    DISCARD = 5
    VISUALIZE = 6


class DataLoggerStates(Enum):
    INITIALIZED = 0
    RECORDING = 1
    NOT_RECORDING = 2


class RobotCommands(Enum):
    POSITION_CTRL = 0
    GRAV_COMP = 1
    RESET = 2
    PREPARE = 3
    OPEN_GRIPPER = 4
    CLOSE_GRIPPER = 5


class RobotStates(Enum):
    INITIALIZED = 0
    READY = 1
