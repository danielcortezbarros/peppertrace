from programming_by_demonstration.skeletal_model.src.others.minimize_jerk_implementation import RealTimeMinimizeJerk
import numpy as np
import json
from programming_by_demonstration.skeletal_model.src.others.minimize_jerk_implementation import JointTrajectoryListener

if __name__ == "__main__":

    # with open("/root/workspace/pepper_rob_ws/src/skeletal_model/config/minimize_jerk_configuration.json") as file:
    #     config=json.load(file)

    # rt_minimizer = RealTimeMinimizeJerk(window_size=config["window_size"], 
    #                                     num_joints=config["num_joints"], 
    #                                     lambda_penalty=config["lambda_penalty"])

    # # Simulated timestamps and angles (for both arms)
    # timestamps = np.array([i for i in range(30)])
    # angles = np.array([[i for i in range(joint_start, joint_start + config["num_joints"])] for joint_start in range(10, 40)])
    # # Update and optimize for each time step
    # for i in range(len(timestamps)):
    #     new_time = timestamps[i]
    #     new_angles = angles[i, :]
    #     optimized_positions = rt_minimizer.optimize_joint_trajectory(new_time, new_angles)
    #     print(f"Optimized positions at time {new_time}: {optimized_positions}")

    config_path = "/root/workspace/pepper_rob_ws/src/skeletal_model/config/minimize_jerk_configuration.json"
    # Path to save the optimized trajectory bag
    output_bag_path = "/root/workspace/demo_data/optimization/optimized_trajectory.bag"

    
    # Initialize the JointTrajectoryListener
    listener = JointTrajectoryListener(config_path, output_bag_path)
    
    # Start the listener node
    try:
        print("Listening...")
        listener.start()
    finally:
        # Ensure the bag is closed when the node is stopped
        listener.close_bag()