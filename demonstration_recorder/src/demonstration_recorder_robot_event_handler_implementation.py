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
        self.robot_state = RobotStates.DISCONNECTED
        self.demonstrate_process = None  # To store the demonstrate process
        self.robot_connection_process = None  # To store the demonstrate process
        self.gui_publisher = publisher
        self.demonstrate_monitor_timer = None
        self.robot_connection_monitor_timer = None
    

    def connect(self, robot_ip: str, port: str, network_interface: str = "eth0") -> bool:
        rospy.loginfo(f"Connecting to {robot_ip}:{port} on {network_interface}...")

        command = [
            "roslaunch",
            "pepper_interface_tests",
            "actuatorTestLaunchRobot.launch",
            f"robot_ip:={robot_ip}",
            f"network_interface:={network_interface}"
        ]
        self.robot_connection_process = launch_ros_subprocess(command)
        if self.robot_connection_process:
            self.robot_connection_monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_robot_connection_process)
            return True
        else:
            return False

    def disconnect(self) -> bool:
        if self.robot_connection_process is not None:
            rospy.loginfo("Disconnecting robot...")
            try:
                self.robot_connection_process.terminate()
                self.robot_connection_process.wait()
                self.robot_connection_process = None
                rospy.loginfo("robot_connection process stopped successfully.")
                self.gui_publisher.publish("[INFO] ROBOT DISCONNECTED")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to stop robot_connection: {str(e)}")
                return False
        else:
            rospy.logwarn("No robot_connection process is running.")
            return False

    def _monitor_robot_connection_process(self, event):
        """
        Periodically checks if the robot_connection process has ended.
        """
        if self.robot_connection_process and self.robot_connection_process.poll() is not None:  # Process has finished
            rospy.loginfo("robot_connection process has died")
            self.gui_publisher.publish("[INFO] ROBOT DISCONNECTED")
            self.robot_connection_process = None
            self.robot_state = RobotStates.DISCONNECTED
            self.robot_connection_monitor_timer.shutdown()


    def start_demonstrate(self) -> bool:
        rospy.loginfo("Starting demonstrate...")

        command = [
            "roslaunch",
            "programming_by_demonstration",
            "skeletal_model.launch"
        ]
        self.demonstrate_process = launch_ros_subprocess(command)

        if self.demonstrate_process:
            self.demonstrate_monitor_timer = rospy.Timer(rospy.Duration(1.0), self._monitor_demonstrate_process)
            return True
        else:
            return False

    def stop_demonstrate(self) -> bool:
        if self.demonstrate_process is not None:
            rospy.loginfo("Stopping demonstrate...")
            try:
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
    
    def _monitor_demonstrate_process(self, event):
        """
        Periodically checks if the demonstrate process has ended.
        """
        if self.demonstrate_process and self.demonstrate_process.poll() is not None:  # Process has finished
            rospy.loginfo("Demonstrate process has died")
            self.gui_publisher.publish("[ERROR] STOPPED DEMONSTRATING due to error in skeletal_model or camera.")
            self.replay_process = None
            self.data_logger_state = DataLoggerStates.IDLE
            self.demonstrate_monitor_timer.shutdown()

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
                information = "[INFO] ROBOT CONNECTED successfully."
            else:
                information = "[ERROR] Failed to connect to robot, please check and try again."

        elif event.command == RobotCommands.DISCONNECT:
            self.disconnect()
            information = "[INFO] ROBOT DISCONNECTED successfully."

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