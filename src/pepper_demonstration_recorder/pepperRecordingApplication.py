#!/usr/bin/env python3
import rospy
from abstract_demo_recorder.DemoRecorder import DemoRecorder
from pepper_demonstration_recorder.pepperInformationDisplay import GuiInfoDisplay
from pepper_demonstration_recorder.pepperRobotEventHandler import PepperEventHandler
from pepper_demonstration_recorder.pepperUserInputHandler import KeyboardInputHandler
from pepper_demonstration_recorder.pepperDataLogger import ROSBagLogger  # Renamed for ROS 1

def main():
    # Initialize the ROS node
    rospy.init_node("demo_recorder_node", anonymous=True)

    # List of topics to record
    pepper_topics = ["lfd_system_logs"]

    # Initialize handlers and logger
    robot_control_handler = PepperEventHandler()
    information_display = GuiInfoDisplay()
    user_input = KeyboardInputHandler()
    data_logger = ROSBagLogger(pepper_topics)  # Renamed to ROSBagLogger for ROS 1

    # Create the DemoRecorder object with the initialized handlers and logger
    demo_recorder = DemoRecorder(
        user_input, robot_control_handler, data_logger, information_display
    )

    # Start recording the demo
    demo_recorder.record_demo()

    # Keep the node alive and responsive
    rospy.spin()

if __name__ == "__main__":
    main()
