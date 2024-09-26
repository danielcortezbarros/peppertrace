#!/usr/bin/env python3
import rospy
import rosbag
import importlib
import sys

class RosBagRecorder:

    def __init__(self, bag_file, topic, queue_size=1000):
        """
        Initialize the recorder with a single topic and bag file.
        """
        self.bag_file = bag_file
        self.topic = topic  # Now we handle a single topic
        self.queue_size = queue_size

    def get_message_class(self, msg_type_str):
        """
        Dynamically import the ROS message class based on the message type string.
        """
        try:
            pkg_name, msg_name = msg_type_str.split('/')
            module = importlib.import_module(f'{pkg_name}.msg')
            return getattr(module, msg_name)
        except (ImportError, AttributeError) as e:
            rospy.logerr(f"Error importing message class for {msg_type_str}: {e}")
            return None

    def run_recording(self):
        """
        Start recording the single topic.
        """
        rospy.init_node('rosbag_logger', anonymous=True)

        # Get the available topics
        available_topics_list = rospy.get_published_topics()
        available_topics = {topic: msg_type for topic, msg_type in available_topics_list}

        if self.topic not in available_topics:
            rospy.logerr(f"Topic {self.topic} not available")
            return

        msg_type_str = available_topics[self.topic]
        msg_class = self.get_message_class(msg_type_str)
        if not msg_class:
            return

        with rosbag.Bag(self.bag_file, 'w') as bag:
            def callback(msg):
                rospy.loginfo(f"Recording message on topic: {self.topic}")
                bag.write(self.topic, msg)

            # Subscribe to the topic and start recording
            rospy.Subscriber(self.topic, msg_class, callback, queue_size=self.queue_size)
            rospy.loginfo(f"Subscribed to {self.topic} with queue size {self.queue_size}")
            rospy.spin()

if __name__ == '__main__':
    # Updated to take only one topic as an argument
    recorder = RosBagRecorder(bag_file=sys.argv[1], topic=sys.argv[2], queue_size=1000)
    recorder.run_recording()