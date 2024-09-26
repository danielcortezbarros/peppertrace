#!/usr/bin/env python3
import rospy
import rospkg
import os
from abstract_demo_recorder.statesAndEvents import DataLoggerStates
from abstract_demo_recorder.abstractDataLogger import AbstractDataLogger
import shutil
import subprocess

class PepperROS1Logger(AbstractDataLogger):
    def __init__(self, topics_list, data_dir, demo_name):
        super().__init__()

        # List of topics to record
        self.topics_list = topics_list
        self.data_dir = data_dir  # Final destination for saved data
        self.demo_name = demo_name
        self.demo_counter = 1  # Counter for demo directories

        # Get the path to the package
        package_path = rospkg.RosPack().get_path('pepper_demonstration_recorder')

        # Construct the path to the config file relative to the package path
        self.rosbag_script_path = os.path.join(package_path, 'src', 'pepper_demonstration_recorder', 'rosBagRecorder.py')

        # State and flags
        self.data_exists = False
        self.status = DataLoggerStates.NOT_RECORDING  # Initial status

        # Recorder for managing bag recording
        self.record_process = None  # Process for running recording

    def prepare_for_logging(self):
        """
        Set up the ROS bag recorder and configure storage options.
        This method is called every time a new directory is created.
        """
        dir_path = os.path.join(self.data_dir, self.demo_name)
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        return dir_path


    def start_logging(self):
        """
        Start recording the ROS bag in a separate process for each topic.
        """
        if self.status == DataLoggerStates.NOT_RECORDING:
            # Prepare the directory to save the bag files
            dir_path = self.prepare_for_logging()

            self.status = DataLoggerStates.RECORDING
            self.record_processes = []  # Store references to processes for each topic

            # Start a recording process for each topic
            for topic in self.topics_list:
                bag_file = os.path.join(dir_path, f"demo_{self.demo_counter}{topic.replace('/', '_')}.bag")

                # Start recording by calling the external script for each topic
                process = subprocess.Popen(['python3', self.rosbag_script_path, bag_file, topic])
                self.record_processes.append(process)  # Track the process for later management

                rospy.loginfo(f"Started recording {topic} in {bag_file}")

            self.data_exists = True
            rospy.loginfo("Started recording all topics")
        else:
            rospy.loginfo("Logger is already recording or there is data to be saved/discarded.")


    def stop_logging(self):
        """
        Stop recording the ROS bag and release resources.
        Gracefully terminate the recording process.
        """
        if self.status == DataLoggerStates.RECORDING:
            rospy.loginfo("Stopping the recording process...")
            for process in self.record_processes:
                process.terminate()  # Terminate the process
                process.wait()  # Wait for the process to fully exit

            self.record_processes = [] # Clear the reference to the process
            self.status = DataLoggerStates.NOT_RECORDING
            rospy.loginfo("Stopped recording successfully")
        else:
            rospy.loginfo("No recording in progress to stop.")

    def save_data(self):
        """
        Save the recorded data.
        """
        if self.data_exists:
            final_path = os.path.join(self.data_dir, self.demo_name, f"demo_{self.demo_counter}.bag")
            rospy.loginfo(f"Data saved to: {final_path}")
            self.demo_counter += 1  # Increment counter for the next demos
            self.data_exists = False
            return True
        else:
            rospy.loginfo("No data to save.")
            return False

    def discard_data(self):
        """
        Discard the recorded data by removing the temporary directory.
        """
        if self.data_exists:
            temp_dir = os.path.join(self.data_dir, self.demo_name, f"demo_{self.demo_counter}.bag")
            try:
                shutil.rmtree(temp_dir)
                rospy.loginfo("Recorded data discarded.")
                self.data_exists = False
                return True
            except Exception as e:
                rospy.logwarn(f"Failed to discard data, file probably does not exist: {e}")
                return False
        else:
            rospy.loginfo("No data to discard.")
            return False

    def reset(self):
        """
        Reset the logger, stop any ongoing logging and discard any recorded data.
        """
        self.stop_logging()
        self.discard_data()
        self.status = DataLoggerStates.NOT_RECORDING
        rospy.loginfo("Data logger reset.")

    def pause_logging(self):
        return super().pause_logging()
