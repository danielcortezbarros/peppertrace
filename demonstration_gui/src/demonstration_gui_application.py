#!/usr/bin/env python3
""" demonstration_gui_application.py Demonstration system that coordinates demonstration and recording process
    
    
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
    /gui/system_logs      | std_msgs.String
    /mediapipe/image_feed | sensor_msgs/Image

    ...
    Published Topics and Message Types

    /gui/commands | std_msgs.String
    ...
    Input Data Files
    demonstration_gui_input.dat
    ...
    Output Data Files
    demonstration_gui_output.dat
    ...
    Configuration Files
    demonstration_gui_configuration.json
    ...
    Example Instantiation of the Module
    rosrun demonstration_gui demonstration_gui_application.py 
    ...
    Author: Daniel Barros, CMU-Africa
    Email: daniel.barros@tum.de
    Date: 21 November, 2024
    Version: v1.0
"""

import sys
import rospkg 
import os
import json
import rospy
import time

#Add implementation modules to python path
package_path = rospkg.RosPack().get_path('programming_by_demonstration')
src_path = os.path.join(package_path, 'demonstration_gui', 'src')
sys.path.insert(0, src_path)

from PyQt5 import QtWidgets, QtCore, QtTest
from demonstration_gui_logic_implementation import MainWindow

def run_unit_tests(main_window):
    """
    Simulates GUI interactions for testing in the main thread.
    """
    rospy.loginfo("Running unit tests...")

    buttons_to_test = {
        "CONNECT": main_window.ui.connectButton,
        "DISCONNECT": main_window.ui.connectButton,
        "START_DEMONSTRATE": main_window.ui.startDemonstrateButton,
        "STOP_DEMONSTRATE": main_window.ui.stopDemonstrateButton,
        "START_RECORD": main_window.ui.startRecordButton,
        "STOP_RECORD": main_window.ui.stopRecordButton,
        "START_REPLAY": main_window.ui.startReplayButton,
        "STOP_REPLAY": main_window.ui.stopReplayButton,
        "BROWSE": main_window.ui.browseButton,
        "CLEAR": main_window.ui.clearLogsButton,
    }

    for name, button in buttons_to_test.items():
        rospy.loginfo(f"Testing {name} button...")

        # Simulate button click
        button.animateClick(200)  # Simulate a button click with animation (200ms duration)
        QtWidgets.QApplication.processEvents()  # Process GUI events to reflect button state
        QtCore.QThread.msleep(1000)  # Small delay to visualize the click

    rospy.loginfo("All tests completed.")
    QtWidgets.QApplication.quit()  # Exit the application after tests


def main():
    rospy.init_node('demonstration_gui_node', anonymous=True)

    # Load configuration
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('programming_by_demonstration')
    config_file_path = os.path.join(package_path, 'demonstration_gui', 'config', 'demonstration_gui_configuration.json')
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    app = QtWidgets.QApplication(sys.argv)

    # Instantiate the GUI
    main_window = MainWindow(
        gui_commands_topic=config["gui_commands_topic"],
        gui_system_logs_topic=config["gui_system_logs_topic"],
        skeletal_model_feed_topic=config["skeletal_model_feed_topic"]
    )
    main_window.show()

    # Check if unit tests should run
    unit_test = rospy.get_param('~unit_test', False)
    rospy.loginfo(f"Performing Unit Test: {unit_test}")

    if unit_test:
        # Use a single-shot timer to delay test execution
        QtCore.QTimer.singleShot(1000, lambda: run_unit_tests(main_window))
    else:
        rospy.loginfo("Starting GUI...")

    # Start the GUI event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()