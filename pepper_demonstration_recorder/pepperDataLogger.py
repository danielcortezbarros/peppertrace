import multiprocessing
import rclpy
from rosbag2_py import Recorder, StorageOptions, RecordOptions
import os
from abstract_demo_recorder.statesAndEvents import DataLoggerStates
from abstract_demo_recorder.abstractDataLogger import AbstractDataLogger
import shutil

class ROS2BagLogger(AbstractDataLogger):
    def __init__(self, topics_list=["lfd_system_logs"], data_dir="/home/dani/Music"):
        super().__init__()

        # Initialize ROS node (for the main process)
        self.node = rclpy.create_node('ros2bag_logger')

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

        self.record_options = RecordOptions()
        self.record_options.all = False  # Do not record all topics
        self.record_options.topics = self.topics_list  # Record only the specified topics
        self.record_options.is_discovery_disabled = True

    def prepare_for_logging(self):
        """
        Set up the ROS bag recorder and configure storage and record options.
        This method is called every time a new directory is created.
        """
        dir = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
        storage_options = StorageOptions(uri=dir, storage_id='sqlite3')
        return storage_options

    def _run_recording(self, storage_options, record_options):
        """
        Function to run the recording in a separate process.
        Create a new ROS 2 node in the new process.
        """
        node = rclpy.create_node('ros2bag_logger_process')  # Create a new ROS 2 node for this process
        node.get_logger().info("Started recording in separate process")
        
        # Create a new Recorder instance and start recording
        recorder = Recorder()
        recorder.record(storage_options, record_options)
    
        
        node.get_logger().info("Finished recording in separate process")

    def start_logging(self):
        """
        Start recording the ROS bag in a separate process.
        """
        if self.status == DataLoggerStates.NOT_RECORDING:
            storage_options = self.prepare_for_logging()  # Prepare storage options
            record_options = self.record_options  # Use the configured record options
            
            self.status = DataLoggerStates.RECORDING
            
            # Run recording in a separate process and pass the options as arguments
            self.record_process = multiprocessing.Process(
                target=self._run_recording, 
                args=(storage_options, record_options)
            )
            self.record_process.start()
            self.data_exists = True
            self.logger.info(f"Started recording")
        else:
            self.logger.info("Logger is already recording or there is data to be saved/discarded.")

    def stop_logging(self):
        """
        Stop recording the ROS bag and release resources.
        Gracefully terminate the recording process.
        """
        if self.status == DataLoggerStates.RECORDING:
            self.logger.info("Stopping the recording process...")
            if self.record_process:
                self.record_process.terminate()  # Terminate the process
                self.record_process.join()  # Wait for the process to fully exit

            self.record_process = None  # Clear the reference to the process
            self.status = DataLoggerStates.NOT_RECORDING
            self.logger.info("Stopped recording successfully")
        else:
            self.logger.info("No recording in progress to stop.")

    def save_data(self):
        """
        Save the recorded data.
        """
        if self.data_exists:
            final_path = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
            self.logger.info(f"Data saved to: {final_path}")
            self.demo_counter += 1  # Increment counter for the next demo
            self.data_exists = False
            return True
        else:
            self.logger.info("No data to save.")
            return False

    def discard_data(self):
        """
        Discard the recorded data by removing the temporary directory.
        """
        if self.data_exists:
            temp_dir = os.path.join(self.data_dir, f"demo_{self.demo_counter}")
            try:
                shutil.rmtree(temp_dir)
                self.logger.info("Recorded data discarded.")
                self.data_exists = False
                return True
            except Exception as e:
                self.logger.error(f"Failed to discard data: {e}")
                return False
        else:
            self.logger.info("No data to discard.")
            return False

    def reset(self):
        """
        Reset the logger, stop any ongoing logging and discard any recorded data.
        """
        self.stop_logging()
        self.discard_data()
        self.status = DataLoggerStates.NOT_RECORDING
        self.logger.info("Data logger reset.")

    def pause_logging(self):
        return super().pause_logging()