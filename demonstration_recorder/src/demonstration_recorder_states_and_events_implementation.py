""" demonstration_recorder_states_and_events_implementation.py Definition of enums for more readability in the components of demonstration recorder

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
    DATA_LOGGING = 0
    ROBOT_CONTROL = 1
    TERMINATE = 2

class DataLoggerCommands(Enum):
    START_RECORD = 0
    STOP_RECORD = 1
    START_REPLAY = 2
    STOP_REPLAY = 3
    SET_TOPICS = 4
    SET_FILTER = 5


class DataLoggerStates(Enum):
    IDLE = 0
    RECORDING = 1
    REPLAYING = 2


class RobotCommands(Enum):
    CONNECT = 0
    DISCONNECT = 1
    START_DEMONSTRATE = 2
    STOP_DEMONSTRATE = 3


class RobotStates(Enum):
    CONNECTED = 0
    DISCONNECTED = 1


class Event:
    def __init__(self, event_type: EventType, command:Union[DataLoggerCommands, RobotCommands]=None, args:dict={}):
        """
        Class constructor. Event is a generic event class that can encapsulate any type of event.

        Args:
            event_type(EventType) : type of event
            command(Union[DataLoggerCommands, RobotCommands) : command corresponding to the event type
            args(dict) : optional arguments for the event callback
        """

        self.event_type = event_type
        self.command = command
        self.args = args



