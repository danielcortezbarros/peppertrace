from abc import ABC, abstractmethod
import logging


class AbstractInformationDisplay(ABC):

    def __init__(self):
        self.logger = logging.getLogger("InformationDisplay")
        self.logger.info("InformationDisplay initialized")
        self.logger.setLevel(logging.DEBUG)
        self.event_queue = None

    @abstractmethod
    def display_input_mapping(self, input_map):
        """
        Display input mapping for user convenience.
        This function is called in the DemoRecorder's __init__ method.
        This function should be implemented according to the user's specific information display setup e.g. GUI.
        """
        raise NotImplementedError

    @abstractmethod
    def display_information(self, information):
        """
        Display system information
        This function can be called by the user anytime.
        This function should be implemented according to the user's specific visualization setup e.g. GUI.
        """
        raise NotImplementedError
    
    def cleanup(self):
        self.logger.info("Exited Information Display")
