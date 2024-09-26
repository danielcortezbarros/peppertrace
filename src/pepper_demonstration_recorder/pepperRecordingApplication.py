#!/usr/bin/env python3
import rospy
import rospkg
import json
import os
from abstract_demo_recorder.DemoRecorder import DemoRecorder
from pepper_demonstration_recorder.pepperInformationDisplay import GuiInfoDisplay
from pepper_demonstration_recorder.pepperRobotEventHandler import PepperEventHandler
from pepper_demonstration_recorder.pepperUserInputHandler import KeyboardInputHandler, MouseInputHandler
from pepper_demonstration_recorder.pepperDataLogger import PepperROS1Logger

def main():
    # Initialize the ROS node
    rospy.init_node("demo_recorder_node", anonymous=True)

    # Get the path to the package
    package_path = rospkg.RosPack().get_path('pepper_demonstration_recorder')

    # Construct the path to the config file relative to the package path
    config_file_path = os.path.join(package_path, 'config', 'recorder_configuration.json')

    # Load the JSON config file
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    print("Loaded config:", config)


    # Initialize handlers and logger
    robot_control_handler = PepperEventHandler()
    information_display = GuiInfoDisplay()
    user_input = MouseInputHandler()
    data_logger = PepperROS1Logger(topics_list=config["data_logger"]["topics_list"],
                                   data_dir=config["data_logger"]["data_dir"],
                                   demo_name=config["data_logger"]["demo_name"],
                                   ) 

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
