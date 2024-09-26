"""Optimizes trajectory for both arms and plots them"""


import numpy as np
from scipy.interpolate import CubicSpline
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.optimize import minimize
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
from pepper_data_processing.bag_reader import get_both_arm_angles_from_bag_file, get_one_arm_angles_from_bag_file, plot_joint_angles

def jerk_cost(t, positions, timestamps):
    cs = InterpolatedUnivariateSpline(timestamps, positions, k=5)
    
    # Calculate the third derivative (jerk) at each timestamp
    jerks = np.array([cs.derivatives(ti)[3] for ti in t])
    
    # Compute the square of the jerk and integrate over time
    jerk_squared = jerks ** 2
    total_jerk = np.trapz(jerk_squared, t)
    
    return total_jerk

def objective(positions, timestamps, original_positions, lambda_penalty):
    jerk_cost_value = jerk_cost(timestamps, positions, timestamps)
    deviation_penalty = lambda_penalty * np.sum((positions - original_positions) ** 2)
    return jerk_cost_value + deviation_penalty

def optimize_joint_trajectory(timestamps, original_positions, lambda_penalty=100):
    print("Lambda penalty: ", lambda_penalty)
    # Use the original positions in the objective function, while optimizing the positions
    result = minimize(objective, original_positions, args=(timestamps, original_positions, lambda_penalty), method='SLSQP')
    optimized_positions = result.x
    
    # Use InterpolatedUnivariateSpline (quintic spline)
    cs = InterpolatedUnivariateSpline(timestamps, optimized_positions, k=5)
    
    # Get velocities (first derivative) and accelerations (second derivative)
    optimized_velocities = np.array([cs.derivatives(t)[1] for t in timestamps])  # First derivative (velocity)
    optimized_accelerations = np.array([cs.derivatives(t)[2] for t in timestamps])  # Second derivative (acceleration)
    
    return optimized_positions, optimized_velocities, optimized_accelerations


def create_and_save_joint_trajectory_msgs(timestamps, positions, velocities, accelerations, joint_names, bag_file_path, topic_name):
    # Open a bag file to save each JointTrajectory message
    with rosbag.Bag(bag_file_path, 'w') as bag:
        # Assume `timestamps` are absolute times from the start of the trajectory
        first_timestamp = timestamps[0]  # Reference point (first timestamp)
        
        for i, t in enumerate(timestamps):
            # Create a new JointTrajectory message for each point
            traj_msg = JointTrajectory()
            traj_msg.joint_names = joint_names

            # Create a JointTrajectoryPoint for the current time step
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            point.positions = positions[i].tolist()  # Set joint positions
            point.velocities = velocities[i].tolist()  # Set joint velocities
            point.accelerations = accelerations[i].tolist()  # Set joint accelerations
            
            traj_msg.points = [point]  # Assign the single point to the message

            # Set the ROS timestamp for this message to match the original trajectory timestamp
            msg_time = rospy.Time(t)  # Convert the trajectory timestamp to ROS time
            
            # Write this single trajectory point to the bag file with the correct timestamp
            bag.write(topic_name, traj_msg, t=msg_time)
        
        print(f"Saved {i+1} messages with proper message timestamps.")

if __name__ == "__main__":
    # Define the bag file path
    joint_states_bag_file_path = "/root/workspace/demo_data/demo09232024/demo_1_joint_states.bag"
    left_arm_bag_file_path = "/root/workspace/demo_data/demo09232024/demo_1_pepper_dcm_LeftArm_controller_command.bag"
    right_arm_bag_file_path = "/root/workspace/demo_data/optimization/optimized_trajectory.bag"

    # Read and parse the bag file
    measured_left_arm_data, measured_right_arm_data = get_both_arm_angles_from_bag_file(joint_states_bag_file_path)
    left_arm_data = get_one_arm_angles_from_bag_file(left_arm_bag_file_path)
    right_arm_data = get_one_arm_angles_from_bag_file(right_arm_bag_file_path)


    # Initialize arrays for the optimized data
    optimized_left_arm_data = np.copy(measured_left_arm_data)
    optimized_right_arm_data = np.copy(measured_right_arm_data)

    # Arrays to store velocities and accelerations
    optimized_left_arm_velocities = np.zeros_like(measured_left_arm_data)
    optimized_right_arm_velocities = np.zeros_like(measured_right_arm_data)
    
    optimized_left_arm_accelerations = np.zeros_like(measured_left_arm_data)
    optimized_right_arm_accelerations = np.zeros_like(measured_right_arm_data)

    # Extract timestamps and joint angles for both arms
    timestamps_left = measured_left_arm_data[:, 0]  # First column is the timestamps
    timestamps_right = measured_right_arm_data[:, 0]

    joint_names_left = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"]
    joint_names_right = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]

    # Optimize and collect positions, velocities, and accelerations for both arms
    for i in range(1, measured_left_arm_data.shape[1]):
        print(f"Optimizing joint {i}...")

        # Optimize left arm joint angles
        optimized_positions, optimized_velocities, optimized_accelerations = optimize_joint_trajectory(timestamps_left, measured_left_arm_data[:, i])
        optimized_left_arm_data[:, i] = optimized_positions
        optimized_left_arm_velocities[:, i] = optimized_velocities
        optimized_left_arm_accelerations[:, i] = optimized_accelerations

        # Optimize right arm joint angles
        optimized_positions, optimized_velocities, optimized_accelerations = optimize_joint_trajectory(timestamps_right, measured_right_arm_data[:, i])
        optimized_right_arm_data[:, i] = optimized_positions
        optimized_right_arm_velocities[:, i] = optimized_velocities
        optimized_right_arm_accelerations[:, i] = optimized_accelerations
        
    # Save each point as its own JointTrajectory message to a bag file
    create_and_save_joint_trajectory_msgs(timestamps_left, 
                                          optimized_left_arm_data[:, 1:], 
                                          optimized_left_arm_velocities[:, 1:],
                                          optimized_left_arm_accelerations[:, 1:], 
                                          joint_names_left, 
                                          "/root/workspace/demo_data/optimization/optimized_left_arm_trajectory.bag", 
                                          '/pepper_dcm/LeftArm_controller/command')
    
    create_and_save_joint_trajectory_msgs(timestamps_right, 
                                          optimized_right_arm_data[:, 1:], 
                                          optimized_right_arm_velocities[:, 1:], 
                                          optimized_right_arm_accelerations[:, 1:], 
                                          joint_names_right, 
                                          "/root/workspace/demo_data/optimization/optimized_right_arm_trajectory.bag", 
                                          '/pepper_dcm/RightArm_controller/command')

    # Now call the plotting function with the original and optimized data
    plot_joint_angles(measured_left_arm_data, measured_right_arm_data, optimized_left_arm_data, optimized_right_arm_data)

