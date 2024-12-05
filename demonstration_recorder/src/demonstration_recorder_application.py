#!/usr/bin/env python3
""" demonstration_recorder_application.py Demonstration system that coordinates recording process
    
    The purpose of this component is to provide a user-friendly demonstration experience. 
    It is made up of several subcomponents: the Data Logger records time series data of 
    the robot's joint angles; the Robot Event Handler manages robot control actions such as 
    connecting and starting and stopping demonstration mode; the User Input Handler processes 
    inputs from a user interface. The Information Display logs and displays system actions. 
    Finally, the Demo Recorder coordinates the overall process, handling user inputs and sending 
    events to the Data Logger or the Robot Event Handler. 
    ...
    Libraries
    Python Libraries
    - sys, os, cv2, json, time, threading, subprocess, importlib, pynput, queue, shutil, enum, typing
    ROS Libraries
    - rospkg, rospy, cv_bridge, std_msgs, rosbag
    ...
    Parameters
    Command-line Parameters
    ...
    Configuration File Parameters

    Parameter | Type
    data_logger.topics_list     | list
    data_logger.data_dir        | str
    data_logger.demo_name       | str

    Subscribed Topics and Message Types
    
    Topic | Type
    /gui/commands                           | std_msgs/String
    /joint_states                           | sensor_msgs/JointState
    /pepper_dcm/LeftArm_controller/command  | trajectory_msgs/JointTrajectory
    /pepper_dcm/RightArm_controller/command | trajectory_msgs/JointTrajectory

    ...
    Published Topics and Message Types

    /gui/system_logs      | std_msgs/String
    /mediapipe/image_feed | sensor_msgs/Image
    ...
    Input Data Files
    demonstration_recorder_input.dat
    ...
    Output Data Files
    demonstration_recorder_output.dat
    ...
    Configuration Files
    demonstration_recorder_configuration.json
    ...
    Example Instantiation of the Module
    rosrun demonstration_recorder demonstration_recorder_application.py 
    ...
    Author: Daniel Barros, CMU-Africa
    Email: daniel.barros@tum.de
    Date: 21 November, 2024
    Version: v1.0
"""

import sys
import os
import rospkg 


#Add implementation modules to python path
package_path = rospkg.RosPack().get_path('programming_from_demonstration')
src_path = os.path.join(package_path, 'demonstration_recorder', 'src')
sys.path.insert(0, src_path)

import json
import rospy

from demonstration_recorder_demo_recorder_implementation import DemoRecorder
from demonstration_recorder_information_display_implementation import GuiInfoDisplay
from demonstration_recorder_robot_event_handler_implementation import PepperRobotEventHandler
from demonstration_recorder_user_input_handler_implementation import GuiInputHandler
from demonstration_recorder_data_logger_implementation import PepperROS1Logger
from std_msgs.msg import String

def main():
    # Initialize the ROS node
    rospy.init_node("demo_recorder_node", anonymous=False)

    # Construct the path to the config file relative to the package path
    config_file_path = os.path.join(package_path, 'demonstration_recorder', 'config', 'demonstration_recorder_configuration.json')

    # Load the JSON config file
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    print("Loaded config:", config)

    gui_publisher=rospy.Publisher(config["gui_system_logs_topic"], String, queue_size=10)


    # Initialize handlers and logger
    robot_control_handler = PepperRobotEventHandler(publisher=gui_publisher)
    information_display = GuiInfoDisplay(publisher=gui_publisher)
    user_input = GuiInputHandler(gui_commands_topic=config["gui_commands_topic"])
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


if __name__ == "__main__":
    main()