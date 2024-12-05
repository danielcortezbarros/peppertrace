""" information_display_implementation.py Sends information via ROS topic to be displayed in the user interface.

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
from std_msgs.msg import String
import time

class GuiInfoDisplay():
    def __init__(self, publisher):
        rospy.loginfo("InformationDisplay initialized")
        self.gui_publisher = publisher

    def display_information(self, information):
        msg = String()
        msg.data = information

        # Publish the message
        self.gui_publisher.publish(msg)

        # Log the message
        rospy.loginfo(f'Displaying: "{msg.data}"')

    def display_input_map(self, input_map):
        msg = String()
        msg.data = 'Map,' + input_map

        # Check if there are any subscribers
        while self.gui_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for subscriber to connect to /gui/system_logs...")
            time.sleep(0.1)
        
        self.gui_publisher.publish(msg)
        rospy.loginfo(f'Published Input Map: "{msg.data}"')


    def cleanup(self):
        rospy.loginfo("Exited Information Display")