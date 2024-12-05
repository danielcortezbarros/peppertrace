""" data_logger_implementation.py Implements control logic for data logging 

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospy
import rospkg

import os
from demonstration_recorder_states_and_events_implementation import DataLoggerStates, DataLoggerCommands
import subprocess

class PepperROS1Logger():
    def __init__(self, topics_list, data_dir, demo_name):
        rospy.loginfo("DataLogger initialized")
        self.data_logger_state = DataLoggerStates.NOT_RECORDING

        # List of topics to record
        self.topics_list = topics_list
        self.demo_dir_path = os.path.join(data_dir, demo_name)
        if not os.path.exists(self.demo_dir_path):
            os.makedirs(self.demo_dir_path)
        self.demo_counter = 1  # Counter for demo directories

        # Get the path to the package
        package_path = rospkg.RosPack().get_path('programming_from_demonstration')

        # Construct the path to the config file relative to the package path
        self.rosbag_script_path = os.path.join(package_path, 'demonstration_recorder', 'src', 'rosbag_recorder_implementation.py')

        # Recorder for managing bag recording
        self.record_processes = None  # Process for running recording


    def start_logging(self):
        """
        Start recording the ROS bag in a separate process for each topic.
        """
        if self.data_logger_state == DataLoggerStates.NOT_RECORDING:

            self.record_processes = []  # Store references to processes for each topic

            # Start a recording process for each topic
            for topic in self.topics_list:
                bag_file = os.path.join(self.demo_dir_path, f"demo_{self.demo_counter}{topic.replace('/', '_')}.bag")

                # Start recording by calling the external script for each topic
                process = subprocess.Popen(['python3', self.rosbag_script_path, bag_file, topic])
                self.record_processes.append(process)  # Track the process for later management

                rospy.loginfo(f"Started recording {topic} in {bag_file}")

            rospy.loginfo("Started recording all topics")
        else:
            rospy.loginfo("Logger is already recording.")


    def stop_logging(self):
        """
        Stop recording the ROS bag and release resources.
        Gracefully terminate the recording process.
        """
        if self.data_logger_state == DataLoggerStates.RECORDING:
            rospy.loginfo("Stopping the recording process...")
            for process in self.record_processes:
                process.terminate()  # Terminate the process
                process.wait()  # Wait for the process to fully exit

            self.record_processes = [] # Clear the reference to the process
            rospy.loginfo("Stopped recording successfully")
        else:
            rospy.loginfo("No recording in progress to stop.")


    def change_topics(self, topics):
        self.topics_list = topics

    def handle_event(self, event):
        """
        Data logging event handler.
        This function is called by the DemoRecorder whenever a Data Logging Event is registered in the queue.
        It is best not to change this function.
        """
        rospy.loginfo("Received event")
        information = ""
        if event.command == DataLoggerCommands.START_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                information = "Data logger is already recording."
            else:
                self.start_logging()
                information = "Data logger is recording now."
                self.data_logger_state = DataLoggerStates.RECORDING

        elif event.command == DataLoggerCommands.STOP_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                information = "Data logger is stopped"
                self.data_logger_state = DataLoggerStates.NOT_RECORDING
            else:
                information = "Data logger is already stopped"

        elif event.command == DataLoggerCommands.CHANGE_TOPICS:
            # set topics_list
            topics = event.args["topics"]
            self.change_topics(topics)
            information = f"Recording topics: {",".join(topics)} "

        else:
            rospy.logwarn("Received unknown command.")
            information = "Data logger received unknown command."
        return information
    

    def cleanup(self):
        """
        Destructor to ensure any resources are properly released.
        """
        self.stop_logging()
        rospy.loginfo("Exited Data Logger")


