#!/usr/bin/env python3
""" demonstration_recorder_replay_implementation.py Implements control logic for data logging 

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""
import sys
import rospy
import rosbag
import rospkg
import os
import subprocess
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#Add implementation modules to python path
package_path = rospkg.RosPack().get_path('programming_by_demonstration')
src_path = os.path.join(package_path, 'skeletal_model', 'src')
sys.path.insert(0, src_path)
from skeletal_model_filters_implementation import DataFilter

def generate_trajectory_message(arm_angles, joint_names, shift_time=2.0):
    """
    Combine trajectory points into a single JointTrajectory message for an arm.

    Args:
        arm_angles (np.ndarray): Filtered trajectory array with columns:
                                 [timestamp, joint1, joint2, ..., jointN].
        joint_names (list): List of joint names for this arm.
        shift_time (float): Time shift (in seconds) for the trajectory start.

    Returns:
        JointTrajectory: Combined trajectory message for the arm.
    """
    # Create a new JointTrajectory message
    trajectory_message = JointTrajectory()
    trajectory_message.header.stamp = rospy.Time.now() + rospy.Duration(shift_time)
    trajectory_message.joint_names = joint_names

    # Get the start time of the first point
    start_time = arm_angles[0, 0]

    # Combine all trajectory points
    for row in arm_angles:
        timestamp = row[0]  # First column is the timestamp
        positions = row[1:]  # Remaining columns are joint positions

        # Create a new trajectory point
        new_point = JointTrajectoryPoint(
            positions=positions.tolist(),
            time_from_start=rospy.Duration(timestamp - start_time + shift_time)  # Adjust relative time
        )
        trajectory_message.points.append(new_point)

    return trajectory_message

# def combine_and_publish_trajectory(bag_file_path, input_topic, output_topic, shift_time=2.0):
#     # Open the bag file
#     with rosbag.Bag(bag_file_path, 'r') as bag:
#         # Get all messages from the specified topic
#         messages = list(bag.read_messages(topics=[input_topic]))

#         if not messages:
#             rospy.logwarn(f"No messages found in topic: {input_topic}")
#             return

#         # Create a new JointTrajectory message
#         combined_trajectory = JointTrajectory()
#         combined_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(shift_time)
#         combined_trajectory.joint_names = messages[0].message.joint_names  # Assuming all messages have the same joint names

#         # Get the start time of the first message in the bag
#         first_msg_time = messages[0].timestamp.to_sec()

#         # Combine all trajectory points
#         for msg_info in messages:
#             msg = msg_info.message
#             for point in msg.points:
#                 # Adjust time_from_start for each point
#                 relative_time = msg_info.timestamp.to_sec() - first_msg_time
#                 new_point = JointTrajectoryPoint(
#                     positions=point.positions,
#                     velocities=point.velocities,
#                     accelerations=point.accelerations,
#                     effort=point.effort,
#                     time_from_start=rospy.Duration(relative_time + shift_time)  # Shift by 2 seconds
#                 )
#                 combined_trajectory.points.append(new_point)

#         # Create a publisher and publish the combined trajectory
#         publisher = rospy.Publisher(output_topic, JointTrajectory, queue_size=10)
#         rospy.sleep(1.0)  # Allow time for the publisher to establish connections
#         publisher.publish(combined_trajectory)
#         rospy.loginfo(f"Published combined trajectory with {len(combined_trajectory.points)} points.")
#         rospy.loginfo(f"Trajectory starts at: {combined_trajectory.header.stamp.to_sec()}")


def get_both_arm_angles_from_bag_file(bag_file):
    """
    Reads the contents of a bag file, extracts joint angles for both arms, and returns them as two NumPy arrays:
    One for the left arm and one for the right arm.
    
    Each array will have the following columns:
    [timestamp, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw]
    """
    # Initialize lists to store joint data and timestamps
    left_data = []
    right_data = []
    time_stamps = []

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == "/joint_states":
                joint_index_map = {name: i for i, name in enumerate(msg.name)}

                # Extract left arm joints
                left_shoulder_pitch = msg.position[joint_index_map["LShoulderPitch"]]
                left_shoulder_roll = msg.position[joint_index_map["LShoulderRoll"]]
                left_elbow_roll = msg.position[joint_index_map["LElbowRoll"]]
                left_elbow_yaw = msg.position[joint_index_map["LElbowYaw"]]
                left_wrist_yaw = msg.position[joint_index_map["LWristYaw"]]

                # Extract right arm joints
                right_shoulder_pitch = msg.position[joint_index_map["RShoulderPitch"]]
                right_shoulder_roll = msg.position[joint_index_map["RShoulderRoll"]]
                right_elbow_roll = msg.position[joint_index_map["RElbowRoll"]]
                right_elbow_yaw = msg.position[joint_index_map["RElbowYaw"]]
                right_wrist_yaw = msg.position[joint_index_map["RWristYaw"]]

                # Append the joint data and timestamp
                time_stamps.append(t.to_sec())
                
                # Left arm: [timestamp, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw]
                left_data.append([t.to_sec(), left_shoulder_pitch, left_shoulder_roll, left_elbow_roll, left_elbow_yaw, left_wrist_yaw])
                
                # Right arm: [timestamp, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw]
                right_data.append([t.to_sec(), right_shoulder_pitch, right_shoulder_roll, right_elbow_roll, right_elbow_yaw, right_wrist_yaw])
    
    # Convert lists to NumPy arrays
    left_data = np.array(left_data)
    right_data = np.array(right_data)

    # Normalize time to start from 0 seconds
    # if data.size > 0:
    #     data[:, 0] -= data[0, 0]  


    return left_data, right_data


def main():
    rospy.init_node("trajectory_combiner", anonymous=True)

    # Get the bag file path and filter type from command-line arguments
    if len(sys.argv) < 3:
        rospy.logerr("Bag file path and filter type are required as arguments.")
        sys.exit(1)

    bag_file_path = sys.argv[1]
    filter_type = sys.argv[2]

    if "unit_test" in bag_file_path:
        rospy.loginfo("Running unit test with rosbag play.")
        replay_process = subprocess.Popen(['rosbag', 'play', bag_file_path])
        return

    # Get measured joint angles
    left_arm_angles, right_arm_angles = get_both_arm_angles_from_bag_file(bag_file_path)

    if left_arm_angles.size == 0 or right_arm_angles.size == 0:
        rospy.logerr("No valid joint states found in the bag file. Exiting.")
        sys.exit(1)

    # Filter the trajectories
    data_filter = DataFilter(filter_type=filter_type)
    filtered_left_arm_angles = data_filter.apply_filter_to_trajectory(trajectory=left_arm_angles, filter_type=filter_type)
    filtered_right_arm_angles = data_filter.apply_filter_to_trajectory(trajectory=right_arm_angles, filter_type=filter_type)

    print(filtered_left_arm_angles)

    # Joint names for each arm
    left_arm_joint_names = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"]
    right_arm_joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]

    # Combine the trajectories into JointTrajectory messages
    left_arm_trajectory = generate_trajectory_message(filtered_left_arm_angles, left_arm_joint_names, shift_time=2.0)
    right_arm_trajectory = generate_trajectory_message(filtered_right_arm_angles, right_arm_joint_names, shift_time=2.0)

    # Create publishers for both arms
    left_arm_publisher = rospy.Publisher("/pepper_dcm/LeftArm_controller/command", JointTrajectory, queue_size=10)
    right_arm_publisher = rospy.Publisher("/pepper_dcm/RightArm_controller/command", JointTrajectory, queue_size=10)

    # Ensure publishers are ready before publishing
    rospy.sleep(1.0)

    # Publish the trajectories
    left_arm_publisher.publish(left_arm_trajectory)
    right_arm_publisher.publish(right_arm_trajectory)

    rospy.loginfo("Trajectory published successfully.")


if __name__ == "__main__":
    main()