import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
import os

class BagReader(Node):
    def __init__(self, db_file):
        super().__init__('bag_reader_node')

        # Check if the file exists
        if not os.path.exists(db_file):
            self.get_logger().error(f"Bag file {db_file} does not exist.")
            return

        # Set up the SequentialReader
        self.reader = SequentialReader()

        # Storage options: point to the bag file
        storage_options = StorageOptions(uri=db_file, storage_id='sqlite3')

        # Converter options: Set the desired serialization format
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

        # Open the bag file
        self.reader.open(storage_options, converter_options)

        # Retrieve the topics and their types from the bag
        topics_and_types = self.reader.get_all_topics_and_types()

        if not topics_and_types:
            self.get_logger().warn(f"No topics were found in the bag file: {db_file}")
        else:
            self.get_logger().info(f"Available topics: {topics_and_types}")

        # Read messages
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()

            # Get the message type dynamically using the topic type
            message_type = get_message(topics_and_types[0].type)
            message = message_type().deserialize(data)

            self.get_logger().info(f"Received message on topic {topic} at time {t}: {message}")


def main(args=None):
    rclpy.init(args=args)

    # Path to the demo_1_0.db3 file
    bag_file = '/home/dani/Music/demo_2/demo_2_0.db3'

    # Create the BagReader node
    bag_reader_node = BagReader(bag_file)

    # Spin the node (process messages)
    rclpy.spin(bag_reader_node)

    # Shutdown when done
    bag_reader_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
