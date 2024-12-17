""" ros_interface_implementation.py Implements ROS interface between Qt GUI and demonstration_recorder

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

class RosThread(QThread):
    update_gui_signal = pyqtSignal(str)
    image_signal = pyqtSignal(np.ndarray)
    publish_signal = pyqtSignal(str)

    def __init__(self, topic_sub, topic_pub, skeletal_model_feed_topic):
        """
        Class constructor. ROSThread implements a separate QThread for the ROS interface, so that the Qt GUI 
        can run in the main application thread.

        Args:
            topic_sub(str): Topic to subscribe to to receive system logs for display in the system logs box
            topic_pub(str): Topic to publish to to send commands to the system 
            skeletal_model_feed_topic(str): Topic to receive images with skeletal model overlay for display
        """

        super().__init__()

        # Initialize the ROS node and connect the publish_signal
        self.ros_node = RosNode(topic_sub, topic_pub, skeletal_model_feed_topic, self.update_gui_signal, self.image_signal)
        self.publish_signal.connect(self.ros_node.publish_message)

    def run(self):
        # Run the ROS event loop
        rospy.spin()

class RosNode:
    def __init__(self, topic_sub, topic_pub, skeletal_model_feed_topic, update_gui_signal, image_signal):
        """
        Class constructor. RosNode contains a ROS node for communication between the GUI the rest of the system.
        Messages are forwarded over a pyqtSignal to the connected pyqtSlots in the MainWindow.

        Args:
            topic_sub(str): Topic to subscribe to to receive system logs for display in the system logs box
            topic_pub(str): Topic to publish to to send commands to the system 
            skeletal_model_feed_topic(str): Topic to receive images with skeletal model overlay for display
            update_gui_signal(pyqtSignal): signal to forward system logs messages to the GUI
            image_signal(pyqtSignal): signal to forward skeletal model images to the GUI
        """

        self.update_gui_signal = update_gui_signal
        self.image_signal = image_signal

        self.bridge=CvBridge()

        # Subscribers
        self.sub = rospy.Subscriber(topic_sub, String, self.sub_callback)
        self.image_sub = rospy.Subscriber(skeletal_model_feed_topic, Image, self.image_callback)

        # Publisher
        self.pub = rospy.Publisher(topic_pub, String, queue_size=10)
        print(type(topic_sub))
        rospy.loginfo(f"Sub to: {topic_sub}")

    def sub_callback(self, msg):
        """Emit the received message to update the GUI"""
        rospy.loginfo("Received message")
        self.update_gui_signal.emit(str(msg.data))

    def image_callback(self, ros_image):
        """Convert ROS image to OpenCV format"""
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
        self.image_signal.emit(cv_image)

    def publish_message(self, msg):
        """Create and publish the message"""
        message = String()
        message.data = msg
        self.pub.publish(message)
        rospy.loginfo(f"Publishing on {self.pub.name}: {msg}")


