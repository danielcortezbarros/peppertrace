from abc import ABC, abstractmethod
import logging
from queue import Queue
from states_and_events import Event, EventType



class AbstractUserInputHandler(ABC):
    def __init__(self):
        self.logger = logging.getLogger("UserInputHandler")
        self.logger.info("UserInputHandler initialized")
        self.logger.setLevel(logging.DEBUG)
        self.event_queue = None

    @abstractmethod
    def process_input(self):
        """
        This function should be used for getting user input and posting events accordingly using post_event().
        This function can be called by user anytime, as long as the input is hooked up.
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def get_input_mapping(self) -> dict:
        """
        'input_mapping' is a necessary property that maps inputs to commands.
        This property should be defined as a dictionary of strings and be returned by this method in your implementation
        """
        raise NotImplementedError

    def set_event_queue(self, event_queue: Queue):
        """Sets the event queue where events will be posted."""

        self.event_queue = event_queue
        self.logger.info("Setting event queue")

    def post_event(self, event_type: EventType, command=None):
        """
        Creates and posts an event to the event queue.

        :param event_type: The type of event, as defined in EventType.
        :param command: The actual event which is the command that should be executed
                        This should come from a Commands Enum defined in states_and_events.py
        """
        if self.event_queue is not None:
            event = Event(event_type, command)
            self.event_queue.put(event)
            self.logger.info(f"Event posted. Type: {event_type}, Command: {command}")
        else:
            self.logger.error("Event queue is not set.")
