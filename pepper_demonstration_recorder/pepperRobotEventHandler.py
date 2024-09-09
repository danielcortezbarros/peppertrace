from abstract_demo_recorder.abstractRobotEventHandler import AbstractRobotEventHandler

class PepperEventHandler(AbstractRobotEventHandler):

    def __init__(self):
        super().__init__()
        
        self.robot = "Pepper"


    def switch_to_position_control(self):
        self.logger.info("Switched to Position Control")
        return

    def switch_to_gravity_compensation(self):
        self.logger.info("Switched to Gravity Compensation")
        return

    def prepare_for_data_recording(self):
        self.logger.info("Prepared for Data Recording")
        return

    def reset_robot(self):
        self.logger.info("Reset Robot")
        return

    def open_gripper(self):
        self.logger.info("Opened Gripper")
        return

    def close_gripper(self):
        self.logger.info("Closed Gripper")
        return