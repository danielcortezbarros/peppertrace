#!/usr/bin/env python3
from abstract_demo_recorder.abstractUserInputHandler import AbstractUserInputHandler
from abstract_demo_recorder.statesAndEvents import EventType, DataLoggerCommands, RobotCommands
from pynput import keyboard

class KeyboardInputHandler(AbstractUserInputHandler):
    def __init__(self, freq=50):
        super().__init__()

        # Use only regular keys for the input mapping
        self.freq = freq
        self.input_mapping = {
            'START': '1',
            'PAUSE': '2',
            'SAVE': '3',
            'SAVE_SNAPSHOT': '4',
            'DISCARD': '5',
            'VISUALIZE': '6',
            'STOP': '0',
            'POSITION': 'p', 
            'GRAV_COMP': 'g',  
            'RESET': 'r',  
            'PREPARE': 'a', 
            'OPEN_GRIPPER': '7',
            'CLOSE_GRIPPER': '8',
        }

        # Start the listener directly without extra threads
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                if key.char == self.input_mapping['START']:
                    self.logger.info("Starting Data Logging")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.START)
                elif key.char == self.input_mapping['PAUSE']:
                    self.logger.info("Pausing Data Logging")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.PAUSE)
                elif key.char == self.input_mapping['SAVE']:
                    self.logger.info("Saving Data")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SAVE)
                elif key.char == self.input_mapping['SAVE_SNAPSHOT']:
                    self.logger.info("Saving Snapshot of Data")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SAVE_SNAPSHOT)
                elif key.char == self.input_mapping['DISCARD']:
                    self.logger.info("Discarding Data")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.DISCARD)
                elif key.char == self.input_mapping['VISUALIZE']:
                    self.logger.info("Visualizing Data")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.VISUALIZE)
                elif key.char == self.input_mapping['STOP']:
                    self.logger.info("Stopping Data Logging")
                    self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.STOP)
                elif key.char == self.input_mapping['POSITION']:
                    self.logger.info("Switching to Position Control")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.POSITION_CTRL)
                elif key.char == self.input_mapping['GRAV_COMP']:
                    self.logger.info("Switching to Gravity Compensation")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.GRAV_COMP)
                elif key.char == self.input_mapping['RESET']:
                    self.logger.info("Resetting Robot")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.RESET)
                elif key.char == self.input_mapping['PREPARE']:
                    self.logger.info("Preparing Robot")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.PREPARE)

            # Handle special keys like Ctrl or others (terminate event)
            elif key == keyboard.Key.ctrl or key == keyboard.Key.esc:  # Add any other termination keys
                self.listener.stop()  # Stop the listener when the special key is pressed
                self.logger.info("Special key pressed, sending terminate event")
                self.post_event(EventType.TERMINATE)


        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if hasattr(key, 'char'):
                if key.char == self.input_mapping['OPEN_GRIPPER']:
                    self.logger.info("Opening Gripper")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.OPEN_GRIPPER)
                elif key.char == self.input_mapping['CLOSE_GRIPPER']:
                    self.logger.info("Closing Gripper")
                    self.post_event(EventType.ROBOT_CONTROL, RobotCommands.CLOSE_GRIPPER)
        except AttributeError as e:
            print(e)

    def get_input_mapping(self) -> dict:
        return self.input_mapping
    
    def process_input(self):
        pass

