from abstract_demo_recorder.abstractInformationDisplay import AbstractInformationDisplay
import rclpy 
from std_msgs.msg import String

class GuiInfoDisplay(AbstractInformationDisplay):
    def __init__(self):
        super().__init__()

        self.node = rclpy.create_node("info_node")
        self.info_publisher = self.node.create_publisher(String, "lfd_system_logs", 10)

        # self.spin_thread = Thread(target=self.spin_node, daemon=True)
        # self.spin_thread.start()

    def display_information(self, information):
        msg = String()
        msg.data = information
        self.info_publisher.publish(msg)
        self.node.get_logger().info(f'Publishing: "{msg.data}"')

    def display_input_mapping(self, input_map):
        msg = String()
        msg.data = 'Map: ' + input_map
        self.info_publisher.publish(msg)
        self.node.get_logger().info(f'Publishing: "{msg.data}"')