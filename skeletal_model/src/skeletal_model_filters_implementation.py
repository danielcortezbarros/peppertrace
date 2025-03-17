""" skeletal_model_filters_implementation.py Implements data filters for retargeted angles

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import rospy
import numpy as np
from std_msgs.msg import String
from scipy import signal


class BiologicalMotionFilter:
    def __init__(self, window_size: int, regularization: float = 1.0, full_traj:bool = False):
        """
        Initialize the Biological Motion Filter.

        Args:
            window_size (int): Number of data points in the sliding window.
            regularization (float): Lambda parameter controlling the trade-off between smoothness and fidelity.
        """
        self.window_size = 0
        self.Q_reg = None
        self.middle_index = self.window_size // 2
        self.lambda_reg = regularization
        self.full_traj = full_traj
        self.set_window_size(window_size=window_size) # sets window_size and Q_reg

        


    def _construct_Q(self, size: int):
        """
        Construct the finite difference matrix (Q) to approximate the third derivative.

        Args:
            size (int): The size of the sliding window.

        Returns:
            np.ndarray: A size x size matrix for the third derivative penalty.
        """
        Q = np.zeros((size, size))
        for i in range(size - 3):
            Q[i, i: i + 4] = [1, -3, 3, -1]  # Third derivative stencil
        return Q.T @ Q  # Q = D3.T @ D3 for smoothness

    def filter(self, window: np.ndarray):
        """
        Apply the Biological Motion Filter to a sliding window.

        Args:
            window (np.ndarray): 2D array of shape (window_size, num_signals), where each column is a signal.

        Returns:
            np.ndarray: The filtered trajectory for each signal (1D array of size num_signals).
        """
        # Right-hand side matrix for all signals
        P = self.lambda_reg * window

        print(f"Q_reg shape: {self.Q_reg.shape}")
        print(f"P shape: {P.shape}")

        # Solve for all signals simultaneously
        filtered_window = np.linalg.solve(self.Q_reg, P)

        # Return the middle row of the filtered trajectories
        if self.full_traj:
            return filtered_window
        else:
            return filtered_window[self.middle_index, :]

    def set_window_size(self, window_size):
        self.window_size = window_size
        Q = self._construct_Q(window_size)
        self.Q_reg = Q + self.lambda_reg * np.eye(self.window_size)

    def set_full_traj(self, full_traj:bool):
        self.full_traj = full_traj


class ButterworthFilter:
    def __init__(self, order: int = 1, cutoff: float = 0.7, sampling_rate: float = 5.3, window_size: int = 5):
        """
        Initialize a Butterworth filter for real-time processing.

        Args:
            order (int): Order of the Butterworth filter.
            cutoff (float): Cutoff frequency of the filter (Hz).
            sampling_rate (float): Sampling rate of the signals (Hz).
            window_size (int): Number of samples in the sliding window.
        """
        self.order = order
        self.cutoff = cutoff
        self.sampling_rate = sampling_rate
        self.window_size = window_size
        self.middle_index = self.window_size // 2

        # Precompute Butterworth filter coefficients
        nyquist = 0.5 * sampling_rate
        normal_cutoff = cutoff / nyquist
        self.b, self.a = signal.butter(order, normal_cutoff, btype="low", analog=False)

        # Initialize filter state
        self.zi = None  # Will be set dynamically when data is available

    def filter(self, window: np.ndarray):
        """
        Apply the Butterworth filter to a given sliding window.

        Args:
            window (np.ndarray): 2D array where each row is a time-ordered data point and 
                                 each column is a signal (e.g., joints). Shape: (window_size, num_signals).

        Returns:
            np.ndarray: The middle values of the filtered signals (1D array of size num_signals).
        """
        if self.zi is None:
            # Initialize the filter state for each signal (column)
            self.zi = signal.lfilter_zi(self.b, self.a).reshape(-1, 1) * window[0:1, :]

        # Apply the Butterworth filter
        filtered_window, self.zi = signal.lfilter(self.b, self.a, window, axis=0, zi=self.zi)

        return filtered_window[self.middle_index, :]


class DataFilter:
    def __init__(self, window_size=5, filter_type="biological", gui_commands_topic="/gui/commands"):
        """
        Initialize the DataFilter class to manage and apply various filters.

        Args:
            window_size (int): Number of data points in the sliding window.
            filter_type (str): Type of filter to apply ("mean", "median", "butterworth", or "biological").
            gui_commands_topic (str): ROS topic name for GUI commands.
        """
        rospy.loginfo("Starting GUI...")
        self.window_size = window_size
        self.window = np.zeros((window_size, 10))
        self.num_valid_points = 0  # Track valid points added
        self.full = False

        self.filter_type = filter_type
        self.valid_filters = ["no", "mean", "median", "biological", "butterworth"]
        gui_sub = rospy.Subscriber(gui_commands_topic, String, self.gui_callback)

        self.butterworth = ButterworthFilter(order=1, cutoff=0.7, sampling_rate=5.3, window_size=self.window_size)
        self.biological = BiologicalMotionFilter(window_size=self.window_size, regularization=1.0)

    def get_filtered_angles(self, data_point: np.ndarray):
        """
        Process the input data point through the selected filter and return filtered values.

        Args:
            data_point (np.ndarray): A single data point (shape: (num_signals,)).

        Returns:
            np.ndarray: Filtered angles (if enough data points are available), otherwise None.
        """

        if self.filter == "no":
            return data_point
        
        self.window[:-1] = self.window[1:]
        self.window[-1] = data_point

        if self.num_valid_points < self.window_size:
            self.num_valid_points += 1

        if self.num_valid_points >= self.window_size:
            self.full = True

        if self.full:
            if self.filter_type == "mean":
                return np.mean(self.window, axis=0)
            elif self.filter_type == "median":
                return np.median(self.window, axis=0)
            elif self.filter_type == "butterworth":
                return self.butterworth.filter(self.window)
            elif self.filter_type == "biological":
                return self.biological.filter(self.window)
            else:
                raise ValueError(f"Filter type '{self.filter_type}' is not supported.")
        else:
            return None

    def gui_callback(self, msg):
        """
        ROS callback for GUI commands.

        Args:
            msg (std_msgs.msg.String): Message containing the filter command.
        """
        if msg.data.startswith("FILTER"):
            self.set_filter_type(msg.data[6:])

    def set_filter_type(self, filter_type):
        """
        Set the filter type for the DataFilter instance.

        Args:
            filter_type (str): The new filter type to apply ("mean", "median", "butterworth", "biological").

        Raises:
            ValueError: If the provided filter type is not recognized.
        """
        if filter_type not in self.valid_filters:
            raise ValueError(f"Invalid filter type '{filter_type}'. Valid options are: {self.valid_filters}")
        self.filter_type = filter_type
        rospy.loginfo(f"Set filter to {filter_type}.")

    def apply_filter_to_trajectory(self, trajectory: np.ndarray, filter_type:str):
        """
        Apply the selected filter to an entire trajectory.

        Args:
            trajectory (np.ndarray): Trajectory data (shape: timesteps x num_signals).
            filter_type (str): Filter to be applied (self.filter_type not used, because this needs to be used in the replay implementation in demonstration_recorder)

        Returns:
            np.ndarray: Filtered trajectory (same shape as input).
        """
        num_timesteps, num_signals = trajectory.shape

        # Apply Butterworth filter to the entire trajectory
        if filter_type == "no":
            return trajectory
        
        elif filter_type == "butterworth":
            # Use scipy's filtfilt for non-causal, zero-phase filtering
            filtered_trajectory = np.zeros_like(trajectory)
            for i in range(num_signals):
                filtered_trajectory[:, i] = signal.filtfilt(self.butterworth.b, self.butterworth.a, trajectory[:, i])
            return filtered_trajectory

        # Apply Biological Motion Filter to the entire trajectory
        elif filter_type == "biological":
            # Use the trajectory length as the window size
            self.biological.set_window_size(trajectory.shape[0])
            self.biological.set_full_traj(True)
            return self.biological.filter(window=trajectory)

        # Apply mean or median filtering along the entire trajectory
        elif filter_type == "mean":
            return np.mean(trajectory, axis=0, keepdims=True).repeat(num_timesteps, axis=0)
        elif filter_type == "median":
            return np.median(trajectory, axis=0, keepdims=True).repeat(num_timesteps, axis=0)

        else:
            raise ValueError(f"Filter type '{self.filter_type}' is not supported.")

            
    

