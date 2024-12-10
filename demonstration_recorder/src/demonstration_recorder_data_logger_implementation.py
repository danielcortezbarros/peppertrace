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
    def __init__(self, data_to_topic_map, data_dir, demo_name, publisher):
        rospy.loginfo("DataLogger initialized")
        self.data_logger_state = DataLoggerStates.IDLE

        # List of topics to record
        self.data_to_topic_map = data_to_topic_map
        self.topics_list = self.set_topics(data_to_topic_map.keys())

        # Check if running unit test
        unit_test = rospy.get_param('~unit_test', False)
        rospy.loginfo(f"Performing Unit Test: {unit_test}")

        if unit_test:
            # Use a single-shot timer to delay test execution
            self.demo_dir_path = os.path.join(data_dir, "unit_test")
        else:
            self.demo_dir_path = os.path.join(data_dir, demo_name)
      
        
        if not os.path.exists(self.demo_dir_path):
            os.makedirs(self.demo_dir_path)
        self.demo_counter = 1  # Counter for demo directories

        # Get the path to the package
        package_path = rospkg.RosPack().get_path('programming_by_demonstration')

        # Construct the path to the config file relative to the package path
        self.rosbag_script_path = os.path.join(package_path, 'demonstration_recorder', 'src', 'demonstration_recorder_rosbag_recorder_implementation.py')

        # Recorder for managing bag recording
        self.record_processes = None  # Process for running recording
        self.replay_processes = None

        self.gui_publisher=publisher

        self.replay_monitor_timer = None


    def start_logging(self):
        """
        Start recording the ROS bag in a separate process for each topic.
        """
        if self.data_logger_state == DataLoggerStates.IDLE:

            self.record_processes = []  # Store references to processes for each topic

            # Start a recording process for each topic
            if len(self.topics_list) !=0:
                for topic in self.topics_list:
                    bag_file = os.path.join(self.demo_dir_path, f"demo_{self.demo_counter}{topic.replace('/', '_')}.bag")

                    # Start recording by calling the external script for each topic
                    process = subprocess.Popen(['python3', self.rosbag_script_path, bag_file, topic])
                    self.record_processes.append(process)  # Track the process for later management

                    rospy.loginfo(f"Started recording {topic} in {bag_file}")

            rospy.loginfo("Started recording all specified topics")
        else:
            rospy.loginfo("Data Logger is busy with recording or replaying.")


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


    def start_replay(self, file_path):
        """
        Start replaying a ROS bag file in a separate process.
        """
        if self.data_logger_state == DataLoggerStates.IDLE:
            if os.path.exists(file_path):
                rospy.loginfo(f"Starting replay of {file_path}")
                self.replay_process = subprocess.Popen(['rosbag', 'play', file_path])
                self.data_logger_state = DataLoggerStates.REPLAYING
                # Monitor the process asynchronously
                self.replay_monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_replay_process)
            else:
                rospy.logwarn(f"Cannot replay. File does not exist: {file_path}")
        else:
            rospy.logwarn("Data Logger is busy with another operation.")


    def stop_replay(self):
        """
        Stop replaying the ROS bag file and release resources.
        """
        if self.data_logger_state == DataLoggerStates.REPLAYING:
            rospy.loginfo("Stopping replay process...")
            self.replay_process.terminate()
            self.replay_process.wait()  # Wait for the process to fully exit
            self.replay_process = None  # Clear the reference to the process
            self.data_logger_state = DataLoggerStates.IDLE
            rospy.loginfo("Replay stopped successfully.")
            self.replay_monitor_timer.shutdown()
        else:
            rospy.loginfo("No replay in progress to stop.")


    def _monitor_replay_process(self, event):
        """
        Periodically checks if the replay process has ended.
        """
        if self.replay_process and self.replay_process.poll() is not None:  # Process has finished
            rospy.loginfo("Replay process completed.")
            self.replay_process = None
            self.data_logger_state = DataLoggerStates.IDLE
            self.replay_monitor_timer.shutdown()


    def set_topics(self, data_list):
        topic_list = []
        for data in data_list:
            if data in self.data_to_topic_map:  # Ensure the data is available as a key in data_to_topic_map
                topics = self.data_to_topic_map[data]
                if isinstance(topics, list):
                    topic_list.extend(topics)  # Add all items in the list
                else:
                    topic_list.append(topics)  # Add single string topics
            else: 
                rospy.logwarn(f"Specified data not available in config: {data}")
        rospy.loginfo(f"Set topics to record: {topic_list}")
        return topic_list

        
    def handle_event(self, event):
        """
        Data logging event handler.
        This function is called by the DemoRecorder whenever a Data Logging Event is registered in the queue.
        It is best not to change this function.
        """
        rospy.loginfo(f"Received event: {event.command}")
        information = ""
        if event.command == DataLoggerCommands.START_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                information = "Data logger is already recording."
            else:
                self.start_logging()
                information = "[INFO] RECORDING successfully."
                self.data_logger_state = DataLoggerStates.RECORDING

        elif event.command == DataLoggerCommands.STOP_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                information = "[INFO] STOPPED RECORDING successfully."
                self.data_logger_state = DataLoggerStates.IDLE
            else:
                information = "Data logger is already stopped"

        elif event.command == DataLoggerCommands.START_REPLAY:
            file_path = event.args.get("file_path")
            if not file_path or not os.path.exists(file_path):
                information = "[WARNING] Cannot replay: path does not exist."
            else:
                self.start_replay(file_path)
                information = f"Started replaying recording: {file_path}"

        elif event.command == DataLoggerCommands.STOP_REPLAY:
            if self.data_logger_state == DataLoggerStates.REPLAYING:
                self.stop_replay()
                information = "Replay stopped."
            else:
                information = "No replay in progress to stop."

        elif event.command == DataLoggerCommands.SET_TOPICS:
            # set topics_list
            topics = event.args["topics"]
            self.topics_list=self.set_topics(topics)
            information = f'Set data to record: {",".join(topics)} '

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


