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
    Simulates GUI interactions for testing.
    """
    rospy.loginfo("Running unit tests...")

    rospy.loginfo("Testing CONNECT button...")
    QtTest.QTest.mouseClick(main_window.ui.connectButton, QtCore.Qt.LeftButton)

    time.sleep(1)

    rospy.loginfo("Testing DISCONNECT button...")
    QtTest.QTest.mouseClick(main_window.ui.connectButton, QtCore.Qt.LeftButton)

    time.sleep(1)

    rospy.loginfo("Testing START_DEMONSTRATE button...")
    QtTest.QTest.mouseClick(main_window.ui.startDemonstrateButton, QtCore.Qt.LeftButton)

    time.sleep(0.5)

    rospy.loginfo("Testing STOP_DEMONSTRATE button...")
    QtTest.QTest.mouseClick(main_window.ui.stopDemonstrateButton, QtCore.Qt.LeftButton)

    time.sleep(0.5)

    rospy.loginfo("Testing START_RECORD button...")
    QtTest.QTest.mouseClick(main_window.ui.startRecordButton, QtCore.Qt.LeftButton)

    time.sleep(0.5)

    rospy.loginfo("Testing STOP_RECORD button...")
    QtTest.QTest.mouseClick(main_window.ui.stopRecordButton, QtCore.Qt.LeftButton)

    time.sleep(0.5)

    rospy.loginfo("Testing START_REPLAY button...")
    QtTest.QTest.mouseClick(main_window.ui.startReplayButton, QtCore.Qt.LeftButton)

    time.sleep(0.5)

    rospy.loginfo("Testing STOP_REPLAY button...")
    QtTest.QTest.mouseClick(main_window.ui.stopReplayButton, QtCore.Qt.LeftButton)

    time.sleep(1)

    rospy.loginfo("Testing BROWSE button...")
    QtTest.QTest.mouseClick(main_window.ui.browseButton, QtCore.Qt.LeftButton)

    time.sleep(1)

    rospy.loginfo("Testing CLEAR button...")
    QtTest.QTest.mouseClick(main_window.ui.clearLogsButton, QtCore.Qt.LeftButton)

    rospy.loginfo("All tests done. Please confirm desired output with deliverable.")


def main():

    # Load configuration
    config_file_path = os.path.join(package_path, 'demonstration_gui', 'config', 'demonstration_gui_configuration.json')
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    app = QtWidgets.QApplication(sys.argv)

    # Instantiate the Qt GUI window
    main_window = MainWindow(gui_commands_topic=config["gui_commands_topic"],
                             gui_system_logs_topic=config["gui_system_logs_topic"],
                             skeletal_model_feed_topic=config["skeletal_model_feed_topic"])
    main_window.show()

    # Run unit tests if the parameter is set
    unit_test = rospy.get_param('/demonstration_gui/unit_test', False)
    if unit_test:
        run_unit_tests(main_window)
    else:
        rospy.loginfo("Opening GUI...")
        sys.exit(app.exec_())


if __name__ == "__main__":
    main()