""" demo_recorder_implementation.py Implements orchestrator of the demonstration system

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

from queue import Queue, Empty
from demonstration_recorder_states_and_events_implementation import EventType
import json
import rospy

class DemoRecorder:
    def __init__(
        self,
        user_input_handler: object,
        robot_event_handler: object,
        data_logger:object,
        information_display:object,
    ):
        rospy.loginfo("DemoRecorder initialized")

        self.event_queue = Queue()
        self.user_input_handler = user_input_handler
        self.robot_event_handler = robot_event_handler
        self.data_logger = data_logger
        self.user_input_handler.set_event_queue(self.event_queue)

        self.information_display = information_display
        input_map = json.dumps(self.user_input_handler.get_input_map())
        self.information_display.display_input_map(input_map)


    def record_demo(self):
        rospy.loginfo("System ready to start recording")

        while not rospy.is_shutdown():
            try:
                event = self.event_queue.get(timeout=0.1)  # Non-blocking get with timeout
            except Empty:
                continue

            if event.event_type == EventType.TERMINATE:
                rospy.loginfo("TERMINATE Event, cleaning up...")
                self.user_input_handler.cleanup()
                self.robot_event_handler.cleanup()
                self.data_logger.cleanup()
                self.information_display.cleanup()
                rospy.loginfo("Recording session terminated")
                break

            if event.event_type == EventType.DATA_LOGGING:
                information = self.data_logger.handle_event(event)
                self.information_display.display_information(f"[Data Logger] {information}")
            elif event.event_type == EventType.ROBOT_CONTROL:
                information = self.robot_event_handler.handle_event(event)
                self.information_display.display_information(f"[Robot] {information}")
            else:
                rospy.logerr(f"Event Type not recognized: {event.event_type}")


