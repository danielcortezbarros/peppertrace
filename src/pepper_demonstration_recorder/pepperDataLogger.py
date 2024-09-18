#!/usr/bin/env python3
import rospy
import rosbag
import os
from abstract_demo_recorder.statesAndEvents import DataLoggerStates
from abstract_demo_recorder.abstractDataLogger import AbstractDataLogger
import shutil
import multiprocessing

class ROSBagLogger(AbstractDataLogger):
    def __init__(self, topics_list=["/lfd_system_logs"], data_dir="/home/dani/Music"):
        super().__init__()

        # List of topics to record
        self.topics_list = topics_list

        # Base directory for storing recorded data
        self.data_dir = data_dir  # Final destination for saved data
        self.demo_counter = 1  # Counter for demo directories

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
        dir_path = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        return dir_path

    def _run_recording(self, bag_file, topics_list):
        """
        Function to run the recording in a separate process.
        Create a new ROS 1 node in the new process.
        """
         # Get all available topics and their message types
        available_topics_list = rospy.get_published_topics()
        
        # Create a dictionary where keys are topic names and values are message types
        available_topics = {topic: msg_type for topic, msg_type in available_topics_list}
        rospy.loginfo(f"Available topics: {available_topics}")

        # Open the bag file for recording
        with rosbag.Bag(bag_file, 'w') as bag:
            def callback(msg, topic):
                """
                Callback to write the message to the bag file.
                """
                rospy.loginfo(f"Recording message on topic: {topic}")
                bag.write(topic, msg)

            # Create subscribers for the topics in topics_list
            subscribers = []
            for topic in topics_list:
                if topic in available_topics:
                    msg_type_str = available_topics[topic]  # Get the message type as a string
                    msg_class = rospy.get_message_class(msg_type_str)  # Convert string to message class

                    if msg_class is not None:
                        # Subscribe to the topic and add a callback for recording
                        subscribers.append(rospy.Subscriber(topic, msg_class, callback, callback_args=topic))
                    else:
                        rospy.logwarn(f"Could not find message class for topic: {topic}")
                else:
                    rospy.logwarn(f"Topic {topic} is not available.")


    def start_logging(self):
        """
        Start recording the ROS bag in a separate process.
        """
        if self.status == DataLoggerStates.NOT_RECORDING:
            # Prepare the directory to save the bag file
            dir_path = self.prepare_for_logging()
            bag_file = os.path.join(dir_path, f"demo_{self.demo_counter}.bag")

            self.status = DataLoggerStates.RECORDING

            # Run recording in a separate process and pass the options as arguments
            self.record_process = multiprocessing.Process(
                target=self._run_recording, 
                args=(bag_file, self.topics_list)
            )
            self.record_process.start()
            self.data_exists = True
            rospy.loginfo("Started recording")
        else:
            rospy.loginfo("Logger is already recording or there is data to be saved/discarded.")

    def stop_logging(self):
        """
        Stop recording the ROS bag and release resources.
        Gracefully terminate the recording process.
        """
        if self.status == DataLoggerStates.RECORDING:
            rospy.loginfo("Stopping the recording process...")
            if self.record_process:
                self.record_process.terminate()  # Terminate the process
                self.record_process.join()  # Wait for the process to fully exit

            self.record_process = None  # Clear the reference to the process
            self.status = DataLoggerStates.NOT_RECORDING
            rospy.loginfo("Stopped recording successfully")
        else:
            rospy.loginfo("No recording in progress to stop.")

    def save_data(self):
        """
        Save the recorded data.
        """
        if self.data_exists:
            final_path = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
            rospy.loginfo(f"Data saved to: {final_path}")
            self.demo_counter += 1  # Increment counter for the next demo
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
            temp_dir = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
            try:
                shutil.rmtree(temp_dir)
                rospy.loginfo("Recorded data discarded.")
                self.data_exists = False
                return True
            except Exception as e:
                rospy.logerr(f"Failed to discard data: {e}")
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
