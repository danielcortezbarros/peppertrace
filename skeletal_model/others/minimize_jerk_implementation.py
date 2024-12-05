import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.optimize import minimize
from scipy.signal import medfilt
import time
import json
import rospy
import rosbag
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from multiprocessing import Pool

def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()  # Record the start time
        result = func(*args, **kwargs)  # Call the actual function
        end_time = time.time()  # Record the end time
        print(f"Function '{func.__name__}' took {end_time - start_time:.6f} seconds to execute.")
        return result
    return wrapper


class OptimizationWindow:
    def __init__(self, window_size, num_joints):

        self.window_size = window_size
        self.current_size = 0

        self.timestamps = np.zeros(window_size)
        self.angles = np.zeros((window_size, num_joints))
        self.optimized_angles = np.zeros((window_size, num_joints))  

    
    def update(self, new_timestamp: float, new_angles: np.ndarray) -> bool:
        if self.current_size < self.window_size:
            self.timestamps[self.current_size] = new_timestamp
            self.angles[self.current_size, :] = new_angles
            self.optimized_angles[self.current_size, :] = new_angles  # Initialize with the new angles
            self.current_size += 1
            print(f"Filling window: {self.current_size}/{self.window_size}")  # Add this print
            return False
        else:
            print("Window full, start optimizing.")  # Add this print when the window is full
            self.timestamps[:-1] = self.timestamps[1:]  
            self.angles[:-1, :] = self.angles[1:, :]  
            self.timestamps[-1] = new_timestamp  
            self.angles[-1, :] = new_angles

            print(f"Updated window timestamps: {self.timestamps[:5]}")
            print(f"Updated window angles: {self.angles[:5]}")
            return True


class RealTimeMinimizeJerk:
    def __init__(self, window_size, num_joints, lambda_penalty):
        self.window = OptimizationWindow(window_size=window_size, num_joints=num_joints)
        self.lambda_penalty = lambda_penalty
        self.num_joints = num_joints

    def objective(positions, *args):
        """
        Objective function for the optimization:
        - Minimizes jerk (third derivative) for each joint
        - Penalizes deviation from the original positions
        """
        timestamps, original_positions, lambda_penalty = args
        cs = InterpolatedUnivariateSpline(timestamps, positions, k=5)
        jerk = cs.derivative(n=3)(timestamps)
        jerk_squared = jerk ** 2
        total_jerk = np.trapz(jerk_squared, timestamps)  # Integral of squared jerk for this joint
        deviation_penalty = lambda_penalty * np.sum((positions - original_positions) ** 2)

        return total_jerk + deviation_penalty


    @staticmethod
    def optimize_single_joint(joint_data):
        """
        Static method to be used for multiprocessing.
        Optimizes a single joint's trajectory.
        """
        timestamps, angles, optimized_angles, lambda_penalty = joint_data

        # Make a copy of optimized_angles to modify for the initial guess
        initial_guess = optimized_angles.copy()
        
        # Modify the initial guess for the last element
        small_increment = 0.01  # You can adjust this value as needed
        initial_guess[-1] = optimized_angles[-2] + small_increment
        
        # Perform optimization using the modified initial guess, but keep the original angles in args
        result = minimize(RealTimeMinimizeJerk.objective, initial_guess, 
                        args=(timestamps, angles, lambda_penalty),  # Use the original angles here
                        method='SLSQP')

        return result.x
    

    @timeit
    def optimize_joint_trajectory(self, new_time, new_angles):
        # Update the window with the latest timestamp and angles for all joints, get current state
        window_full = self.window.update(new_time, new_angles)

        if window_full == False:
            print("Window is not full yet, waiting for more data.")
            return
        else:
            print("Window full, starting optimization.")
            joint_data = []
            for joint in range(self.num_joints):
                # Prepare the data for each joint to be passed to the multiprocessing pool
                joint_data.append((
                    self.window.timestamps,
                    self.window.angles[:, joint],
                    self.window.optimized_angles[:, joint],
                    self.lambda_penalty
                ))
                print(f"Joint {joint} data before optimization:")
                print(f"Timestamps: {self.window.timestamps}")
                print(f"Angles: {self.window.angles[:, joint]}")
                print(f"Optimized angles: {self.window.optimized_angles[:, joint]}")

            print("Starting multiprocessing optimization for all joints.")
            # Use multiprocessing to optimize each joint in parallel
            with Pool(processes=self.num_joints) as pool:
                optimized_results = pool.map(RealTimeMinimizeJerk.optimize_single_joint, joint_data)

            # Update the optimized angles with the results
            for joint, result in enumerate(optimized_results):
                print(f"Optimized angles for joint {joint}: {result}")
                self.window.optimized_angles[:, joint] = result

            print(f"Returning optimized positions: {self.window.optimized_angles[-1, :]}")
            return self.window.optimized_angles[-1, :]
            

class JointTrajectoryListener:
    def __init__(self, config_path, output_bag_path):
        # Load configuration
        with open(config_path) as file:
            self.config = json.load(file)
        
        # Initialize RealTimeMinimizeJerk
        self.rt_minimizer = RealTimeMinimizeJerk(window_size=self.config["window_size"], 
                                                 num_joints=self.config["num_joints"], 
                                                 lambda_penalty=self.config["lambda_penalty"])
        

        # Initialize ROS subscriber
        rospy.init_node('joint_trajectory_listener', anonymous=False)
        rospy.Subscriber("/pepper_dcm/LeftArm_controller/command", JointTrajectory, self.callback)

        # Open the output bag file to save optimized trajectories
        self.bag = rosbag.Bag(output_bag_path, 'w')
        self.start_time = rospy.Time.now().to_sec()

    def callback(self, msg):
        if msg.points:
            # Extract joint angles from the incoming JointTrajectory message
            angles = np.array(msg.points[0].positions)  # Get the joint positions
 
            # Get the timestamp from the message header
            new_time = rospy.Time.now().to_sec()-self.start_time
            print("Time: ", new_time)

            # Optimize the joint trajectory
            try:
                optimized_positions = self.rt_minimizer.optimize_joint_trajectory(new_time, angles)
            except Exception as e:
                print(f"Error during optimization: {e}")
                return

            if optimized_positions is not None:
                # Get the last element of the optimized trajectory (the most recent command)
                last_optimized_position = optimized_positions

                # Create a new JointTrajectory message with the last optimized position
                optimized_msg = JointTrajectory()
                optimized_msg.header.stamp = msg.header.stamp  # Use the same timestamp from the original message
                optimized_msg.joint_names = msg.joint_names  # Copy the joint names

                # Create a JointTrajectoryPoint
                optimized_point = JointTrajectoryPoint()
                optimized_point.positions = last_optimized_position.tolist()  # Set the last optimized joint positions
                optimized_point.time_from_start = rospy.Duration(0.033)  # Keep the same 33 ms interval

                # Add the point to the trajectory message
                optimized_msg.points.append(optimized_point)

                # Write the optimized message to the bag
                self.bag.write('/pepper_dcm/LeftArm_controller/command', optimized_msg)

                # You would also send this command to the robot’s motors here (if needed)
                print(f"Sending optimized command at time {new_time}: {last_optimized_position}")

    def start(self):
        # Spin to keep the script running and listening to ROS messages
        rospy.spin()

    def close_bag(self):
        # Close the bag file when done
        self.bag.close()

if __name__ == "__main__":
    # Path to your config file
    config_path = "/root/workspace/pepper_rob_ws/src/skeletal_model/config/minimize_jerk_configuration.json"
    # Path to save the optimized trajectory bag
    output_bag_path = "/root/workspace/pepper_rob_ws/src/skeletal_model/optimized_trajectory.bag"
    
    # Initialize the JointTrajectoryListener
    listener = JointTrajectoryListener(config_path, output_bag_path)
    
    # Start the listener node
    try:
        listener.start()
    finally:
        # Ensure the bag is closed when the node is stopped
        listener.close_bag()


