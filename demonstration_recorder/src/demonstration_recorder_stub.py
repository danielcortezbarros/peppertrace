#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import String

file_path = ""

def callback(msg):
    global file_path

    if msg.data.startswith("/"):
        file_path = msg.data
    rospy.loginfo(msg.data)

def main():
    # Initialize the node
    global file_path

    rospy.init_node('demonstration_recorder_unit_test', anonymous=False)
    
    # Publishers and Subscribers
    gui_commands_pub = rospy.Publisher('/gui/commands', String, queue_size=10)
    unit_test_pub = rospy.Publisher('/demonstration_recorder/unit_test', String, queue_size=10)
    system_logs_sub = rospy.Subscriber("/gui/system_logs", String, callback)
    
    # Wait for connections
    rospy.loginfo("Waiting for subscribers...")
    rospy.sleep(2)  # Allow time for connections
    
    try:
        # Step 1: Publish "RECORD[/demonstration_recorder/unit_test]" to /gui/commands
        gui_commands_pub.publish("RECORD['unit_test']")
        rospy.loginfo("Published: RECORD['/demonstration_recorder/unit_test']")
        rospy.sleep(1)

        # Step 2: Publish "START_RECORD" to /gui/commands
        gui_commands_pub.publish("START_RECORD")
        rospy.loginfo("Published: START_RECORD")
        rospy.sleep(1)

        # Step 3: Publish "Unit Test Message 1", "Unit Test Message 2", "Unit Test Message 3" to /demonstration_recorder/unit_test
        test_messages = ["Unit Test Message 1", "Unit Test Message 2", "Unit Test Message 3"]
        for message in test_messages:
            unit_test_pub.publish(message)
            rospy.loginfo(f"Published: {message}")
            rospy.sleep(1)

        # Step 4: Publish "STOP_RECORD" to /gui/commands
        gui_commands_pub.publish("STOP_RECORD")
        rospy.loginfo("Published: STOP_RECORD")
        rospy.sleep(2)

        # Step 5: Publish "START_REPLAY,/root/workspace/demo_data/unit_test/demo_1_demonstration_recorder_unit_test.bag" to /gui/commands
        rospy.loginfo(f"Replaying {file_path}")
        replay_command = f"START_REPLAY,{file_path}"
        gui_commands_pub.publish(replay_command)
        rospy.loginfo(f"Published: {replay_command}")
        rospy.sleep(4)

        rospy.signal_shutdown("Unit test completed. Shutting down skeletal_model_stub.")


    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception occurred.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
