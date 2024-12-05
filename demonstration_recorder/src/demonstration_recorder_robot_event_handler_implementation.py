""" robot_event_handler_implementation.py Implements control logic for robot events

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
import subprocess
from demonstration_recorder_states_and_events_implementation import RobotStates, RobotCommands
import time
import threading


def launch_ros_subprocess(command: list):
    try:
        rospy.loginfo("Starting roslaunch process...")
        process = subprocess.Popen(command, stdout=None, stderr=None)
        if process.poll() is None:
            rospy.loginfo("ROS launch process started successfully.")
            return process
        else:
            rospy.logerr("Failed to start ROS launch process.")
            return None
    except Exception as e:
        rospy.logerr(f"Error: {str(e)}")
        return None
    

class PepperRobotEventHandler():

    def __init__(self, publisher):
        rospy.loginfo("RobotEventHandler initialized")
        self.robot_state = RobotStates.READY
        self.demonstrate_process = None  # To store the demonstrate process
        self.monitor_thread = None  # Thread for monitoring the demonstrate process
        self.monitoring = False  # Flag to control monitoring
        self.gui_publisher = publisher

    def connect(self, robot_ip: str, port: str, network_interface: str = "eth0") -> bool:
        rospy.loginfo(f"Connecting to {robot_ip}:{port} on {network_interface}...")

        command = [
            "roslaunch",
            "pepper_interface_tests",
            "actuatorTestLaunchRobot.launch",
            f"robot_ip:={robot_ip}",
            f"network_interface:={network_interface}"
        ]
        process = launch_ros_subprocess(command)
        return process is not None

    def start_demonstrate(self) -> bool:
        rospy.loginfo("Starting demonstrate...")

        command = [
            "roslaunch",
            "programming_from_demonstration",
            "skeletal_model.launch"
        ]
        self.demonstrate_process = launch_ros_subprocess(command)

        if self.demonstrate_process:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self.monitor_demonstrate_process)  # No daemon=True
            self.monitor_thread.start()
            return True
        return False

    def stop_demonstrate(self) -> bool:
        if self.demonstrate_process is not None:
            rospy.loginfo("Stopping demonstrate...")
            try:
                self.monitoring = False  # Stop monitoring
                if self.monitor_thread.is_alive():
                    self.monitor_thread.join()  # Wait for the thread to finish
                self.demonstrate_process.terminate()
                self.demonstrate_process.wait()
                self.demonstrate_process = None
                rospy.loginfo("demonstrate process stopped successfully.")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to stop demonstrate: {str(e)}")
                return False
        else:
            rospy.logwarn("No demonstrate process is running.")
            return False

    def monitor_demonstrate_process(self):
        """
        Monitors the demonstrate process and logs a message if it dies unexpectedly.
        """
        rospy.loginfo("Starting demonstrate process monitoring.")
        while self.monitoring:
            if self.demonstrate_process.poll() is not None:  # Process has terminated
                rospy.logwarn("Demonstrate process has died.")
                self.monitoring = False
                self.demonstrate_process = None
                self.gui_publisher.publish("[ERROR] STOPPED DEMONSTRATING due to error in skeletal_model or camera.")
            time.sleep(1)  # Check the process status every second
        rospy.loginfo("Stopped demonstrate process monitoring.")
    
    def handle_event(self, event):
        """
        Robot Control event handler.
        This function is called by the DemoRecorder whenever a Robot Control Event is registered in the queue.
        It is best not to change this function.
        """
        information = ""
        if event.command == RobotCommands.CONNECT:
            connected = self.connect(robot_ip=event.args["robot_ip"], port=event.args["port"])
            if connected:
                information = "[INFO] CONNECTED successfully."
            else:
                information = "[ERROR] Failed to connect to robot, please check and try again."

        elif event.command == RobotCommands.START_DEMONSTRATE:
            success = self.start_demonstrate()
            if success:
                information = "[INFO] DEMONSTRATING successfully."
            else:
                information = "[ERROR] Failed to start demonstrate, please check and try again."

        elif event.command == RobotCommands.STOP_DEMONSTRATE:
            success = self.stop_demonstrate()
            if success:
                information = "[INFO] STOPPED DEMONSTRATING successfully."
            else:
                information = "[ERROR] Failed to stop demonstrate, please check and try again."

        else:
            rospy.logwarn("Received unknown command.")
            information = "Robot received unknown command."
        return information
    
    def cleanup(self):
        rospy.loginfo("Exited Robot Event Handler")