#!/usr/bin/env python

import rospy
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def combine_and_publish_trajectory(bag_file_path, input_topic, output_topic, shift_time=2.0):
    # Open the bag file
    with rosbag.Bag(bag_file_path, 'r') as bag:
        # Get all messages from the specified topic
        messages = list(bag.read_messages(topics=[input_topic]))

        if not messages:
            rospy.logwarn(f"No messages found in topic: {input_topic}")
            return

        # Create a new JointTrajectory message
        combined_trajectory = JointTrajectory()
        combined_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(shift_time)
        combined_trajectory.joint_names = messages[0].message.joint_names  # Assuming all messages have the same joint names

        # Get the start time of the first message in the bag
        first_msg_time = messages[0].timestamp.to_sec()

        # Combine all trajectory points
        for msg_info in messages:
            msg = msg_info.message
            for point in msg.points:
                # Adjust time_from_start for each point
                relative_time = msg_info.timestamp.to_sec() - first_msg_time
                new_point = JointTrajectoryPoint(
                    positions=point.positions,
                    velocities=point.velocities,
                    accelerations=point.accelerations,
                    effort=point.effort,
                    time_from_start=rospy.Duration(relative_time + shift_time)  # Shift by 2 seconds
                )
                combined_trajectory.points.append(new_point)

        # Create a publisher and publish the combined trajectory
        publisher = rospy.Publisher(output_topic, JointTrajectory, queue_size=10)
        rospy.sleep(1.0)  # Allow time for the publisher to establish connections
        publisher.publish(combined_trajectory)
        rospy.loginfo(f"Published combined trajectory with {len(combined_trajectory.points)} points.")
        rospy.loginfo(f"Trajectory starts at: {combined_trajectory.header.stamp.to_sec()}")

if __name__ == "__main__":
    rospy.init_node("trajectory_combiner", anonymous=True)

    # Bag file path (update this to your actual bag file path)
    bag_file_path = "/root/workspace/demo_data/banana/demo2.bag"

    # Input topic and output topic
    input_topic = "/pepper_dcm/LeftArm_controller/command"
    output_topic = "/pepper_dcm/LeftArm_controller/command"

    try:
        combine_and_publish_trajectory(bag_file_path, input_topic, output_topic, shift_time=2.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("Trajectory combiner interrupted.")