from queue import Queue
import logging
from abstract_demo_recorder.statesAndEvents import (
    Event,
    EventType,
    DataLoggerCommands,
    RobotCommands,
)
from abstract_demo_recorder.abstractDataLogger import AbstractDataLogger
from abstract_demo_recorder.abstractRobotEventHandler import AbstractRobotEventHandler
from abstract_demo_recorder.abstractUserInputHandler import AbstractUserInputHandler
from abstract_demo_recorder.abstractInformationDisplay import AbstractInformationDisplay
import json

logging.basicConfig(
    level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(name)s - %(message)s"
)


class DemoRecorder:
    def __init__(
        self,
        user_input_handler: AbstractUserInputHandler,
        robot_event_handler: AbstractRobotEventHandler,
        data_logger: AbstractDataLogger,
        information_display: AbstractInformationDisplay,
    ):
        self.logger = logging.getLogger("DemoRecorder")
        self.logger.info("DemoRecorder initialized")
        self.logger.setLevel(logging.DEBUG)

        self.event_queue = Queue()
        self.user_input_handler = user_input_handler
        self.robot_event_handler = robot_event_handler
        self.data_logger = data_logger
        self.user_input_handler.set_event_queue(self.event_queue)

        self.information_display = information_display
        input_map = json.dumps(self.user_input_handler.get_input_mapping())
        self.information_display.display_input_mapping(input_map)

    def record_demo(self):
        """
        Main demonstration recording function which implements an event loop that processes data logging and robot
        control events from a queue
        """
        self.logger.info("System ready to start recording")

        while True:
            event = self.event_queue.get()  # Get the next event from the queue

            if event.event_type == EventType.TERMINATE:
                # Terminate the loop when encountering a termination event
                self.user_input_handler.cleanup()
                self.robot_event_handler.cleanup()
                self.data_logger.cleanup()
                self.information_display.cleanup()
                self.logger.info("Recording session terminated")
                break

            if event.event_type == EventType.DATA_LOGGING:
                data_logger_state, information = self.data_logger.handle_event(event)
                data_logger_information = "Data Logger: " + information
                self.information_display.display_information(data_logger_information)
            elif event.event_type == EventType.ROBOT_CONTROL:
                robot_state, information = self.robot_event_handler.handle_event(event)
                robot_information = "Robot: " + information
                self.information_display.display_information(robot_information)
            else:
                self.logger.error(f"Event Type not recognized: {event.event_type}")

