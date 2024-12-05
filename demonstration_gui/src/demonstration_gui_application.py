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

#Add implementation modules to python path
package_path = rospkg.RosPack().get_path('programming_from_demonstration')
src_path = os.path.join(package_path, 'demonstration_gui', 'src')
sys.path.insert(0, src_path)

from PyQt5 import QtWidgets
from demonstration_gui_logic_implementation import MainWindow


def main():
    # Construct the path to the config file relative to the package path
    config_file_path = os.path.join(package_path, 'demonstration_gui', 'config', 'demonstration_gui_configuration.json')

    # Load the JSON config file
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    app = QtWidgets.QApplication(sys.argv)
    # Instantiate the Qt GUI window
    main_window = MainWindow(gui_commands_topic=config["gui_commands_topic"],
                             gui_system_logs_topic=config["gui_system_logs_topic"],
                             skeletal_model_feed_topic=config["skeletal_model_feed_topic"])
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
