""" states_and_events_implementation.py Definition of enums for more readability in the components of demonstration recorder

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

from enum import Enum
from typing import Union


class EventType(Enum):
    """Generic event types."""

    DATA_LOGGING = 0
    ROBOT_CONTROL = 1
    TERMINATE = 2

class DataLoggerCommands(Enum):
    START_RECORD = 0
    STOP_RECORD = 1
    SET_TOPICS = 2


class DataLoggerStates(Enum):
    RECORDING = 1
    NOT_RECORDING = 2


class RobotCommands(Enum):
    CONNECT = 0
    START_DEMONSTRATE = 1
    STOP_DEMONSTRATE = 2


class RobotStates(Enum):
    READY = 0
    NOT_READY = 1


class Event:
    """A generic event class that can encapsulate any type of event."""

    def __init__(self, event_type: EventType, command:Union[DataLoggerCommands, RobotCommands]=None, args:dict={}):
        """
        Creates an event with specified information.
        :param event_type: Type of the event (DATA_LOGGING or ROBOT_CONTROL)
        :param command: Event
        :param info: Information string
        """
        self.event_type = event_type
        self.command = command
        self.args = args



