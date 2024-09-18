#!/usr/bin/env python3
from abstract_demo_recorder.abstractInformationDisplay import AbstractInformationDisplay
import rospy
from std_msgs.msg import String

class GuiInfoDisplay(AbstractInformationDisplay):
    def __init__(self):
        super().__init__()

        # Create a publisher
        self.info_publisher = rospy.Publisher("lfd_system_logs", String, queue_size=10)

    def display_information(self, information):
        msg = String()
        msg.data = information

        # Publish the message
        self.info_publisher.publish(msg)

        # Log the message
        rospy.loginfo(f'Publishing: "{msg.data}"')

    def display_input_mapping(self, input_map):
        msg = String()
        msg.data = 'Map: ' + input_map

        # Publish the message
        self.info_publisher.publish(msg)

        # Log the message
        rospy.loginfo(f'Publishing: "{msg.data}"')