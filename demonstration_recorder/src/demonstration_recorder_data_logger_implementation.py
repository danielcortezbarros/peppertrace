""" demonstration_recorder_data_logger_implementation.py Implements control logic for data logging 

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
import re  
from demonstration_recorder_states_and_events_implementation import DataLoggerStates, DataLoggerCommands
import subprocess

class PepperROS1Logger():
    def __init__(self, data_to_topic_map, data_dir, demo_name, publisher):
        """
        Class constructor. PepperROS1Logger contains methods for managing recording and replaying demonstrations.

        Args:
            data_to_topic_map(dict): Dictionary that maps types of data to the topics they can be recorded from
            data_dir(str): Base directory where the demonstration data should be stored
            demo_name(str): Directory where set of demonstrations should be stored
            publisher(rospy.Publisher): Publisher to send messages to the GUI directly
        """

        rospy.loginfo("DataLogger initialized")
        self.data_logger_state = DataLoggerStates.IDLE

        # List of topics to record
        self.data_to_topic_map = data_to_topic_map
        self.topics_list = self.set_topics(data_to_topic_map.keys())

        # Check if running unit test
        unit_test = rospy.get_param('~unit_test', False)
        rospy.loginfo(f"Performing Unit Test: {unit_test}")

        # Define the demo directory path, folder structure is data_dir -> demo_dir_path -> demo_name1.bag
        self.data_dir = data_dir
        if unit_test:
            self.demo_name="unit_test"
        else:
            self.demo_name=demo_name

        self.demo_dir_path = os.path.join(self.data_dir, self.demo_name)
        
        if not os.path.exists(self.demo_dir_path):
            os.makedirs(self.demo_dir_path)

        self.record_process = None  
        self.replay_process = None
        self.replay_monitor_timer = None
        self.gui_publisher = publisher


    def find_next_demo_number(self):
        """
        Finds the next available demo number for the current demo_name in the demo directory.
        Returns:
            int: Next available demo number.
        """
        existing_files = os.listdir(self.demo_dir_path)
        max_demo_number = 0

        # Regex to match demo files with the current demo name (e.g., arms_up1.bag)
        demo_file_pattern = re.compile(fr"{self.demo_name}(\d+)\.bag")
        for file in existing_files:
            match = demo_file_pattern.match(file)
            if match:
                demo_number = int(match.group(1))
                if demo_number > max_demo_number:
                    max_demo_number = demo_number
        
        # Return the next available demo number
        return max_demo_number + 1


    def start_logging(self):
        """Start recording the ROS bag with the demo name and number included in the file name."""
        if self.data_logger_state == DataLoggerStates.IDLE:
            if len(self.topics_list) != 0:
                # Create a new bag file name with demo name and counter
                bag_file = os.path.join(self.demo_dir_path, f"{self.demo_name}{self.find_next_demo_number()}.bag")
                rospy.loginfo(f"Recording topics: {self.topics_list}")

                # Build the rosbag record command
                # rosbag_command = ["rosbag", "record", "-O", bag_file] + self.topics_list
                rosbag_command = ["rosbag", "record", "-O", bag_file] + ["/test_topic"]

                # Start the recording process
                self.record_process = subprocess.Popen(rosbag_command)
                rospy.loginfo(f"Started recording topics in {bag_file}")

                # Update state and increment demo counter
                self.data_logger_state = DataLoggerStates.RECORDING
                
            else:
                rospy.logwarn("No topics specified to record.")
        else:
            rospy.loginfo("Data Logger is busy with recording or replaying.")


    def stop_logging(self):
        """
        Stop recording the ROS bag and release resources.
        Gracefully terminate the recording process.
        """

        if self.data_logger_state == DataLoggerStates.RECORDING:
            rospy.loginfo("Stopping the recording process...")
            if self.record_process:
                self.record_process.terminate()  # Terminate the process
                self.record_process.wait()  # Wait for the process to fully exit
                self.record_process = None  # Clear the reference

            self.gui_publisher.publish(os.path.join(self.demo_dir_path, f"{self.demo_name}{str(self.find_next_demo_number()-1)}.bag"))
            self.data_logger_state = DataLoggerStates.IDLE
            rospy.loginfo("Stopped recording successfully")
        else:
            rospy.loginfo("No recording in progress to stop.")

    def start_replay(self, file_path):
        """ Start replaying a ROS bag file in a separate process."""

        if self.data_logger_state == DataLoggerStates.IDLE:
            if os.path.exists(file_path):
                rospy.loginfo(f"Starting replay of {file_path}")
                self.replay_process = subprocess.Popen([
                    'rosrun', 
                    'programming_by_demonstration', 
                    'demonstration_recorder_replay_implementation.py', 
                    file_path
                ])
                self.data_logger_state = DataLoggerStates.REPLAYING
                # Monitor the process asynchronously
                self.replay_monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_replay_process)
            else:
                rospy.logwarn(f"Cannot replay. File does not exist: {file_path}")
        else:
            rospy.logwarn("Data Logger is busy with another operation.")


    def stop_replay(self):
        """ Stop replaying the ROS bag file and release resources."""

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
        """ Periodically check if the replay process has ended."""

        if self.replay_process and self.replay_process.poll() is not None:  # Process has finished
            rospy.loginfo("Replay process completed.")
            self.replay_process = None
            self.data_logger_state = DataLoggerStates.IDLE
            self.replay_monitor_timer.shutdown()


    def set_topics(self, data_list):
        """ Set the topics which ought to be recorded by extracting them from the data_to_topic_map. """

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


    def set_topics(self, data_list):
        """ Set the topics which ought to be recorded by extracting them from the data_to_topic_map. """

        topic_list = []
        for data in data_list:
            if data in self.data_to_topic_map:
                topics = self.data_to_topic_map[data]
                if isinstance(topics, list):
                    topic_list.extend(topics)
                else:
                    topic_list.append(topics)
            else:
                rospy.logwarn(f"Specified data not available in config: {data}")
        rospy.loginfo(f"Set topics to record: {topic_list}")
        return topic_list

    def handle_event(self, event):
        """
        Data logging event handler.
        This function is called by the DemoRecorder whenever a Data Logging Event is registered in the queue.
        """

        rospy.loginfo(f"Received event: {event.command}")
        information = ""
        if event.command == DataLoggerCommands.START_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                information = "Data logger is already recording."
            else:
                demo_name = event.args["demo_name"]
                self.update_demo_name(demo_name)
                self.start_logging()
                information = f"[INFO] RECORDING {demo_name} successfully."
                self.data_logger_state = DataLoggerStates.RECORDING

        elif event.command == DataLoggerCommands.STOP_RECORD:
            if self.data_logger_state == DataLoggerStates.RECORDING:
                self.stop_logging()
                information = "[INFO] STOPPED RECORDING successfully."
                self.data_logger_state = DataLoggerStates.IDLE
            else:
                information = "Data logger is already stopped"

        elif event.command == DataLoggerCommands.START_REPLAY:
            # Construct the full path
            demo_name = os.path.join(event.args.get("demo_name"))
            base_demo_name = re.sub(r'\d+$', '', demo_name) #remove digits
            file_path = os.path.join(self.data_dir, base_demo_name, f"{demo_name}.bag")
            
            if not file_path or not os.path.exists(file_path):
                information = f"[WARNING] Cannot replay: {file_path} does not exist."
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

    def update_demo_name(self, demo_name):
        #folder structure is data_dir -> demo_dir_path -> demo_name1.bag
        self.demo_name=demo_name
        self.demo_dir_path = os.path.join(self.data_dir, self.demo_name)
        if not os.path.exists(self.demo_dir_path):
            os.makedirs(self.demo_dir_path)

    def cleanup(self):
        """ Destructor to ensure any resources are properly released."""
        self.stop_logging()
        rospy.loginfo("Exited Data Logger")
