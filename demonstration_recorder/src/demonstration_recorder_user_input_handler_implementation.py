""" demonstration_recorder_user_input_handler_implementation.py Contains implementations of input handlers for different devices

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

from demonstration_recorder_states_and_events_implementation import Event, EventType, DataLoggerCommands, RobotCommands
#from pynput import keyboard, mouse
import rospy
from queue import Queue
from std_msgs.msg import String
from threading import Thread


class GuiInputHandler:
    def __init__(self, gui_commands_topic:str):
        """
        Class constructor. GuiInputHandler processes user input from the GUI buttons. 

        Args:
            gui_commands_topic(str): topic to subscribe to to receive GUI commands.
        """

        rospy.loginfo("UserInputHandler initialized")
        self.event_queue = None

        # Define input map
        self.input_map = {
            'Demonstrate': {
                'Start': 'Use GUI button',
                'Stop': 'Use GUI button'
            },
            'Record': {
                'Start': 'Use GUI button',
                'Stop': 'Use GUI button'
            }
        }

        # Initialize the subscriber to /gui/commands
        self.subscriber = rospy.Subscriber(gui_commands_topic, String, self.gui_command_callback)
        rospy.loginfo(f"Subscribed to {gui_commands_topic}")

        # Start ROS spinning in a separate thread
        self.ros_thread = Thread(target=self.run_ros_spin)
        self.ros_thread.start()

    def run_ros_spin(self):
        rospy.loginfo("Starting ROS spin in a separate thread")
        rospy.spin()

    def gui_command_callback(self, msg):
        """
        Callback for processing GUI commands by posting the appropriate events on the GUI for the DemoRecorder to process.

        Args:
            msg(std_msgs.String): ROS String message containing GUI command
        """
        command = msg.data
        rospy.loginfo(f"Received GUI Command: {command}")

        # Post the received command as an event
        if command.startswith("CONNECT"):
            _, robot_ip, port = command.split(",")
            self.post_event(EventType.ROBOT_CONTROL, RobotCommands.CONNECT, args={'robot_ip': robot_ip, 'port': port})
        elif command == "START_DEMONSTRATE":
            self.post_event(EventType.ROBOT_CONTROL, RobotCommands.START_DEMONSTRATE)
        elif command == "STOP_DEMONSTRATE":
            self.post_event(EventType.ROBOT_CONTROL, RobotCommands.STOP_DEMONSTRATE)
        elif command.startswith("START_RECORD"):
            _, demo_name = command.split(",")
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.START_RECORD, args={'demo_name':demo_name})
        elif command == "STOP_RECORD":
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.STOP_RECORD)
        elif command.startswith("RECORD"):
            topics = eval(command[len("RECORD"):])  # Convert the string to a list
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SET_TOPICS, args={"topics":topics})
        elif command.startswith("START_REPLAY"):
            _, demo_name = command.split(",")
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.START_REPLAY, args={'demo_name': demo_name})
        elif command == "STOP_REPLAY":
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.STOP_REPLAY)
        elif command.startswith("FILTER"):
            filter_type=command[6:]
            self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SET_FILTER, args={'filter_type': filter_type})
        else:
            rospy.logwarn(f"Unknown GUI command: {command}")

    def get_input_map(self) -> dict:
        return self.input_map

    def set_event_queue(self, event_queue: Queue):
        """Sets the event queue where events will be posted."""

        self.event_queue = event_queue
        rospy.loginfo("Event queue set")

    def post_event(self, event_type: EventType, command=None, args: dict = {}):
        """
        Creates and posts events to the event queue. 

        Args:
            event_type(EventType) : type of event
            command(Union[DataLoggerCommands, RobotCommands) : command corresponding to the event type
            args(dict) : optional arguments for the event callback
        """

        if self.event_queue is not None:
            event = Event(event_type, command, args)
            self.event_queue.put(event)
            rospy.loginfo(f"Event posted. Type: {event_type}, Command: {command}")
        else:
            rospy.logerr("Event queue is not set.")

    def cleanup(self):
        rospy.loginfo("Exited Input Handler")




# class KeyboardInputHandler():
#     def __init__(self, freq=50):
#         rospy.loginfo("UserInputHandler initialized")
#         self.event_queue = None

#         # Use only regular keys for the input map
#         self.freq = freq
#         self.input_map = {
#             'START': '1',
#             'PAUSE': '2',
#             'SAVE': '3',
#             'SAVE_SNAPSHOT': '4',
#             'DISCARD': '5',
#             'VISUALIZE': '6',
#             'STOP': '0',
#             'POSITION': 'p', 
#             'GRAV_COMP': 'g',  
#             'RESET': 'r',  
#             'PREPARE': 'a', 
#             'OPEN_GRIPPER': '7',
#             'CLOSE_GRIPPER': '8',
#         }

#         # Start the listener directly without extra threads
#         self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
#         self.listener.start()

#     def on_press(self, key):
#         try:
#             if hasattr(key, 'char') and key.char is not None:
#                 if key.char == self.input_map['START']:
#                     rospy.loginfo("Starting Data Logging")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.START)
#                 elif key.char == self.input_map['PAUSE']:
#                     rospy.loginfo("Pausing Data Logging")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.PAUSE)
#                 elif key.char == self.input_map['SAVE']:
#                     rospy.loginfo("Saving Data")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SAVE)
#                 elif key.char == self.input_map['SAVE_SNAPSHOT']:
#                     rospy.loginfo("Saving Snapshot of Data")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SAVE_SNAPSHOT)
#                 elif key.char == self.input_map['DISCARD']:
#                     rospy.loginfo("Discarding Data")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.DISCARD)
#                 elif key.char == self.input_map['VISUALIZE']:
#                     rospy.loginfo("Visualizing Data")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.VISUALIZE)
#                 elif key.char == self.input_map['STOP']:
#                     rospy.loginfo("Stopping Data Logging")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.STOP)
#                 elif key.char == self.input_map['POSITION']:
#                     rospy.loginfo("Switching to Position Control")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.POSITION_CTRL)
#                 elif key.char == self.input_map['GRAV_COMP']:
#                     rospy.loginfo("Switching to Gravity Compensation")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.GRAV_COMP)
#                 elif key.char == self.input_map['RESET']:
#                     rospy.loginfo("Resetting Robot")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.RESET)
#                 elif key.char == self.input_map['PREPARE']:
#                     rospy.loginfo("Preparing Robot")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.PREPARE)

#             # Handle special keys like Ctrl or others (terminate event)
#             elif key == keyboard.Key.ctrl or key == keyboard.Key.esc:  # Add any other termination keys
#                 self.listener.stop()  # Stop the listener when the special key is pressed
#                 rospy.loginfo("Special key pressed, sending terminate event")
#                 self.post_event(EventType.TERMINATE)


#         except AttributeError:
#             pass

#     def on_release(self, key):
#         try:
#             if hasattr(key, 'char'):
#                 if key.char == self.input_map['OPEN_GRIPPER']:
#                     rospy.loginfo("Opening Gripper")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.OPEN_GRIPPER)
#                 elif key.char == self.input_map['CLOSE_GRIPPER']:
#                     rospy.loginfo("Closing Gripper")
#                     self.post_event(EventType.ROBOT_CONTROL, RobotCommands.CLOSE_GRIPPER)
#         except AttributeError as e:
#             print(e)

#     def get_input_map(self) -> dict:
#         return self.input_map

#     def set_event_queue(self, event_queue: Queue):
#         """Sets the event queue where events will be posted."""

#         self.event_queue = event_queue
#         rospy.loginfo("Setting event queue")

#     def post_event(self, event_type: EventType, command=None):
#         """
#         Creates and posts an event to the event queue.

#         :param event_type: The type of event, as defined in EventType.
#         :param command: The actual event which is the command that should be executed
#                         This should come from a Commands Enum defined in states_and_events.py
#         """
#         if self.event_queue is not None:
#             event = Event(event_type, command)
#             self.event_queue.put(event)
#             rospy.loginfo(f"Event posted. Type: {event_type}, Command: {command}")
#         else:
#             rospy.logerr("Event queue is not set.")

#     def cleanup(self):
#         rospy.loginfo("Exited Input Handler")



# class MouseInputHandler():
#     def __init__(self, freq=50):
#         rospy.loginfo("UserInputHandler initialized")
#         self.event_queue = None

#         # Use only regular keys for the input map
#         self.freq = freq
#         self.input_map = {
#             'START': 'LEFT',
#             'SAVE': 'LEFT AGAIN',
#         }

#         # Start the listener directly without extra threads
#         self.listener = mouse.Listener(on_click=self.on_click)
#         self.listener.start()
#         rospy.loginfo("Started mouse listener")

#         self.recording = False

#     def on_click(self, x, y, button, pressed):
#         if pressed:
#             if button == mouse.Button.left:
#                 if self.recording == False:
#                     rospy.loginfo("Starting Data Logging")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.START)
#                     self.recording = True
#                 else:
#                     rospy.loginfo("Saving Data")
#                     self.post_event(EventType.DATA_LOGGING, DataLoggerCommands.SAVE)
#                     self.recording = False
#             elif button == mouse.Button.button9:
#                 rospy.loginfo("Stop button pushed. Terminating listener.")
#                 self.listener.stop() 
#                 self.post_event(EventType.TERMINATE)


#     def get_input_map(self) -> dict:
#         return self.input_map

#     def set_event_queue(self, event_queue: Queue):
#         """Sets the event queue where events will be posted."""

#         self.event_queue = event_queue
#         rospy.loginfo("Setting event queue")

#     def post_event(self, event_type: EventType, command=None):
#         """
#         Creates and posts an event to the event queue.

#         :param event_type: The type of event, as defined in EventType.
#         :param command: The actual event which is the command that should be executed
#                         This should come from a Commands Enum defined in states_and_events.py
#         """
#         if self.event_queue is not None:
#             event = Event(event_type, command)
#             self.event_queue.put(event)
#             rospy.loginfo(f"Event posted. Type: {event_type}, Command: {command}")
#         else:
#             rospy.logerr("Event queue is not set.")

#     def cleanup(self):
#         rospy.loginfo("Exited Input Handler")