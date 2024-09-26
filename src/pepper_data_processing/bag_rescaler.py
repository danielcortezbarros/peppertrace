import rosbag
import rospy

# Input and output bag paths
input_bag_path = '/root/workspace/demo_data/demo09232024/demo_1_pepper_dcm_LeftArm_controller_command.bag'
output_bag_path = '/root/workspace/demo_data/demo09232024/rescaled_demo_1_pepper_dcm_LeftArm_controller_command.bag'

# Open the original bag file
with rosbag.Bag(input_bag_path, 'r') as inbag:
    with rosbag.Bag(output_bag_path, 'w') as outbag:
        initial_time = None
        new_time = rospy.Time(0)

        # Iterate over all messages in the bag
        for topic, msg, t in inbag.read_messages():
            if initial_time is None:
                initial_time = t

            # Calculate the new time offset for each message (1-second increments)
            new_time = rospy.Time(new_time.to_sec() + 0.4)  # Add 1 second for each message

            # Optionally update the message's header timestamp (if it exists)
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                msg.header.stamp = new_time

            # Write the message with the new time to the output bag
            outbag.write(topic, msg, new_time)

print("New bag file with slowed-down message rate saved to:", output_bag_path)