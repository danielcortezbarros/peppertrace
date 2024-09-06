from abc import ABC, abstractmethod
import logging
from states_and_events import DataLoggerCommands, DataLoggerStates


class AbstractDataLogger(ABC):
    def __init__(self):
        self.logger = logging.getLogger("DataLogger")
        self.logger.info("DataLogger initialized")
        self.logger.setLevel(logging.DEBUG)
        self.data_logger_state = DataLoggerStates.INITIALIZED

    @abstractmethod
    def prepare_for_logging(self):
        """
        Prepare the logging system.
        Initialize necessary processes and variables.
        """
        raise NotImplementedError

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @abstractmethod
    def start_logging(self):
        """
        This function should implement start of the logging process.
        You can implement this by saving already in final location or logging to a tempfile and using save to log to
        the permanent data directory.
        """
        raise NotImplementedError

    @abstractmethod
    def stop_logging(self):
        raise NotImplementedError

    @abstractmethod
    def pause_logging(self):
        self.logger.warning("Pausing not supported by the logger.")

    @abstractmethod
    def save_data(self) -> bool:
        """
        Saves data logged e.g. in tempfile to a permanent location
        This function should be called after the logger has started.
        This is function is called by handle_event on SAVE and SAVE_SNAPSHOT events, the difference is that in
        SAVE_SNAPSHOT the recording is not stopped

        returns: True if data was saved, False if there was no data to save

        """
        raise NotImplementedError

    @abstractmethod
    def discard_data(self) -> bool:
        """D
        Discards data for example from a tempfile

        returns: True if data was saved, False if there was no data to save
        """
        raise NotImplementedError

    def visualize_data(self) -> bool:
        self.logger.info("Visualization function is not implemented.")
        return False

    def handle_event(self, event):
        """
        Data logging event handler.
        This function is called by the DemoRecorder whenever a Data Logging Event is registered in the queue.
        It is best not to change this function.
        """
        self.logger.info("Received event")
        information = ""
        if event.command == DataLoggerCommands.START:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                information = "Data logger is already recording."
            else:
                self.start_logging()
                information = "Data logger is recording now."
                self.data_logger_state = DataLoggerStates.RECORDING

        elif event.command == DataLoggerCommands.STOP:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                information = "Data logger is stopped"
                self.data_logger_state = DataLoggerStates.NOT_RECORDING
            else:
                information = "Data logger is already stopped"

        elif event.command == DataLoggerCommands.PAUSE:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.pause_logging()
            else:
                information = "Data logger is not recording anything to pause."
        elif event.command == DataLoggerCommands.SAVE:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                self.visualize_data()
                self.save_data()
                self.data_logger_state = DataLoggerStates.NOT_RECORDING
                information = "Data logger was stopped and data was saved."
            else:
                self.visualize_data()
                saved = self.save_data()

                if saved:
                    information = "Data was saved."
                else:
                    information = "No data to save"

        elif event.command == DataLoggerCommands.SAVE_SNAPSHOT:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.save_data()
                information = "Data snapshot was saved without stopping the recording."
            else:
                information = "Data logger is not running, saving snapshot is not possible."

        elif event.command == DataLoggerCommands.DISCARD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                self.discard_data()
                self.data_logger_state = DataLoggerStates.NOT_RECORDING
                information = "Data logger was stopped, data was discarded."
            else:
                discarded = self.discard_data()
                if discarded:
                    information = "Data was discarded."
                else:
                    information = "No data to discard"

        elif event.command == DataLoggerCommands.VISUALIZE:
            if self.visualize_data():
                information = "Data was visualized."
            else:
                information = "Problem occurred while visualizing data."
        else:
            self.logger.warning("Received unknown command.")
            information = "Data logger received unknown command."
        return self.data_logger_state, information
