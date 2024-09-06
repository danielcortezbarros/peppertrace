from abc import ABC, abstractmethod
import logging
from states_and_events import RobotStates, RobotCommands

class AbstractRobotEventHandler(ABC):
    def __init__(self):
        """
        Initialize the needed objects, including robot here.
        Perform initial robot functions which should be done only once.
        Motions that are needed to be performed at init should be performed here.
        """
        self.logger = logging.getLogger("RobotEventHandler")
        self.logger.info("RobotEventHandler initialized")
        self.logger.setLevel(logging.DEBUG)
        self.robot_state = RobotStates.READY

    @abstractmethod
    def switch_to_position_control(self):
        raise NotImplementedError

    @abstractmethod
    def switch_to_gravity_compensation(self):
        raise NotImplementedError

    @abstractmethod
    def open_gripper(self):
        raise NotImplementedError

    @abstractmethod
    def close_gripper(self):
        raise NotImplementedError

    @abstractmethod
    def prepare_for_data_recording(self):
        """
        Prepare robot for data recording.
        This function can be called by user anytime.
        This function is called on demand by user
        This function is meant for recording sensor biases, offsets, etc.
            e.g. doing local tau offset for SARA.
        """
        raise NotImplementedError

    @abstractmethod
    def reset_robot(self):
        """
        This function should be used for resetting robot and clearing cached information.
        This function can be called by user anytime.
        This function should be used for resetting any cached information in this class.
        For example, sensor bias, offsets, etc.
        :return:
        """
        raise NotImplementedError

    def handle_event(self, event):
        """
        Robot Control event handler.
        This function is called by the DemoRecorder whenever a Robot Control Event is registered in the queue.
        It is best not to change this function.
        """
        information = ""
        if event.command == RobotCommands.GRAV_COMP:
            self.switch_to_gravity_compensation()
            information = "Robot switched to Gravity Compensation mode."
        elif event.command == RobotCommands.POSITION_CTRL:
            self.switch_to_position_control()
            information = "Robot switched to Position Control mode"
        elif event.command == RobotCommands.OPEN_GRIPPER:
            self.open_gripper()
            information = "Opening Gripper"
        elif event.command == RobotCommands.CLOSE_GRIPPER:
            self.close_gripper()
            information = "Closing gripper"
        elif event.command == RobotCommands.RESET:
            self.reset_robot()
            information = "Robot reset completed."
        elif event.command == RobotCommands.PREPARE:
            self.prepare_for_data_recording()
            information = "Robot is prepared for data recording."
        else:
            self.logger.warning("Received unknown command.")
            information = "Robot received unknown command."
        return self.robot_state, information
