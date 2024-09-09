import rclpy
from abstract_demo_recorder.DemoRecorder import DemoRecorder
from pepper_demonstration_recorder.pepperInformationDisplay import GuiInfoDisplay
from pepper_demonstration_recorder.pepperRobotEventHandler import PepperEventHandler
from pepper_demonstration_recorder.pepperUserInputHandler import KeyboardInputHandler 
from pepper_demonstration_recorder.pepperDataLogger import ROS2BagLogger
from threading import Thread

def main():

    rclpy.init(args=None)
    pepper_topics = ["lfd_system_logs"]

    robot_control_handler = PepperEventHandler()
    data_logger = ROS2BagLogger(pepper_topics)
    information_display = GuiInfoDisplay()
    user_input = KeyboardInputHandler()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(information_display.node)
   # executor.add_node(user_input.node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    demo_recorder = DemoRecorder(
        user_input, robot_control_handler, data_logger, information_display
    )
    demo_recorder.record_demo()

    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()