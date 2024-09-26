#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import numpy as np

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
    # if left_data.size > 0:
    #     left_data[:, 0] -= left_data[0, 0]  # Normalize time for left arm
    # if right_data.size > 0:
    #     right_data[:, 0] -= right_data[0, 0]  # Normalize time for right arm

    return left_data, right_data


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

def get_one_arm_angles_from_bag_file(bag_file):
    """
    Reads the contents of a bag file, extracts joint angles, and returns them.
    """
    # Initialize lists to store joint data and timestamps
    data = []
    time_stamps = []

    # Open the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():

            if topic == "/pepper_dcm/LeftArm_controller/command" or topic == "/pepper_dcm/RightArm_controller/command":

                # Append left arm angles
                shoulder_pitch = msg.points[0].positions[0]
                shoulder_roll = msg.points[0].positions[1]
                elbow_roll = msg.points[0].positions[2]
                elbow_yaw = msg.points[0].positions[3]
                wrist_yaw = msg.points[0].positions[4]

                header_time = msg.header.stamp.to_sec()
                # You can append this timestamp instead of t
                time_stamps.append(header_time)
                # Left arm: [timestamp, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw]
                data.append([header_time, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw])

    data = np.array(data)

    # Normalize time to start from 0 seconds
    # if data.size > 0:
    #     data[:, 0] -= data[0, 0]  


    return data


def plot_joint_angles(left_arm_data, right_arm_data=None, measured_left_arm_data=None, measured_right_arm_data=None, label1="1", label2="2"):
    """
    Plots the joint angles for both commanded and measured data from the left and right arms.
    
    All input arrays are NumPy arrays with the following columns:
    [timestamp, shoulder_pitch, shoulder_roll, elbow_roll, elbow_yaw, wrist_yaw].
    """

    # Normalize timestamps by subtracting the first element of the left_arm_data timestamps
    start_time = left_arm_data[0, 0]  # First timestamp of the left_arm_data
    left_arm_data[:, 0] -= start_time
    

    # Create a figure with 5 subplots for each joint angle
    fig, axs = plt.subplots(2, 5, figsize=(18, 8))  # 2 rows, 5 columns for each joint

    fig.suptitle(f'Joint Angles - {label1} vs {label2}', fontsize=16)
    manager = plt.get_current_fig_manager()
    manager.window.showMaximized()

    # Joint labels for titles
    joint_labels = ['Shoulder Pitch', 'Shoulder Roll', 'Elbow Roll', 'Elbow Yaw', 'Wrist Yaw']

    # Plot for each joint (left and right arm)
    for i in range(5):
        # Plot left arm joints 
        axs[0, i].plot(left_arm_data[:, 0], left_arm_data[:, i+1], label=label1, color='b')
        axs[0, i].set_title(f'Left Arm {joint_labels[i]}')

    if right_arm_data is not None:
        right_arm_data[:, 0] -= start_time
        for i in range(5):
            axs[1, i].plot(right_arm_data[:, 0], right_arm_data[:, i+1], label=label1, color='b')
            axs[1, i].set_title(f'Right Arm {joint_labels[i]}')
        axs[1,0].legend()
        

    if measured_left_arm_data is not None:
        measured_left_arm_data[:, 0] -= measured_left_arm_data[0,0]
        for i in range(5):
            axs[0, i].plot(measured_left_arm_data[:, 0], measured_left_arm_data[:, i+1], label=label2, color='r')

    if measured_right_arm_data is not None:
        measured_right_arm_data[:, 0] -= start_time
        for i in range(5):
            axs[1, i].plot(measured_right_arm_data[:, 0], measured_right_arm_data[:, i+1], label=label2, color='r')


    axs[0,0].legend()
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Angle (rad)')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Angle (rad)')

    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0, 0, 1, 0.95])  # Leave space for the title
    plt.show()

def plot_angles_one_joint(timestamps, positions, optimized_positions):
    plt.plot(timestamps, positions, label="Original Trajectory", linestyle='--', color='red')
    plt.plot(timestamps, optimized_positions, label="Optimized Trajectory", color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('LShoulderRoll Angle')
    plt.legend()
    plt.title('Minimizing Jerk for LShoulderRoll')
    plt.show()

   
if __name__ == '__main__':
    # Define the bag file path
    joint_states_bag_file_path = "/root/workspace/demo_data/demo09232024/demo_1_joint_states.bag"
    left_arm_bag_file_path = "/root/workspace/demo_data/demo09232024/demo_1_pepper_dcm_LeftArm_controller_command.bag"
    right_arm_bag_file_path = "/root/workspace/demo_data/optimization/optimized_trajectory.bag"

    # Read and parse the bag file
    measured_left_arm_data, measured_right_arm_data = get_both_arm_angles_from_bag_file(joint_states_bag_file_path)
    left_arm_data = get_one_arm_angles_from_bag_file(left_arm_bag_file_path)
    right_arm_data = get_one_arm_angles_from_bag_file(right_arm_bag_file_path)

    print("Left shape: ", left_arm_data.shape)
    print("Right shape: ", right_arm_data.shape)


    # Plot the joint angles
    plot_joint_angles(left_arm_data=left_arm_data, measured_left_arm_data=measured_left_arm_data)