# import rospy
import numpy as np
from std_msgs.msg import String
import rospy
    
from scipy import signal


import numpy as np

class BiologicalMotionFilter:
    def __init__(self, window_size: int, regularization: float = 1.0):
        """
        Initialize the Biological Motion Filter.
        
        Parameters:
        - window_size: int
            Number of data points in the sliding window.
        - regularization: float
            The lambda parameter controlling the trade-off between smoothness and fidelity.
        """
        self.window_size = window_size
        self.middle_index = self.window_size // 2
        self.lambda_reg = regularization

        # Construct the finite difference matrix for the third derivative (Q)
        Q = self._construct_Q(window_size)
        # Regularization matrix (Q + lambda * I)
        self.Q_reg = Q + self.lambda_reg * np.eye(self.window_size)

    def _construct_Q(self, size: int):
        """
        Construct the finite difference matrix (Q) to approximate the third derivative.

        Parameters:
        - size: int
            The size of the sliding window.

        Returns:
        - Q: np.ndarray
            A size x size matrix for the third derivative penalty.
        """
        Q = np.zeros((size, size))
        for i in range(size - 3):
            Q[i, i : i + 4] = [1, -3, 3, -1]  # Third derivative stencil
        return Q.T @ Q  # Q = D3.T @ D3 for smoothness

    def filter(self, window: np.ndarray):
        """
        Apply the Biological Motion Filter to a sliding window.

        Parameters:
        - window: np.ndarray
            2D array of shape (window_size, num_signals), where each column is a signal.

        Returns:
        - filtered_window: np.ndarray
            The filtered trajectory for each signal (1D array of size num_signals).
        """


        # Right-hand side matrix for all signals
        P = self.lambda_reg * window

        # Solve for all signals simultaneously
        filtered_window = np.linalg.solve(self.Q_reg, P)

        # Return the middle row of the filtered trajectories
        return filtered_window[self.middle_index, :]




class ButterworthFilter:
    def __init__(self, order: int = 1, cutoff: float = 0.7, sampling_rate: float = 5.3, window_size: int = 5):
        """
        Initialize a Butterworth filter for real-time processing.
        
        Parameters:
        - order: int
            Order of the Butterworth filter.
        - cutoff: float
            Cutoff frequency of the filter (Hz).
        - sampling_rate: float
            Sampling rate of the signals (Hz).
        - window_size: int
            Number of samples in the sliding window.
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
        
        Parameters:
        - window: np.ndarray
            2D array where each row is a time-ordered data point and each column is a signal (e.g., joints).
            Shape: `(window_size, num_signals)`.
        
        Returns:
        - np.ndarray
            The middle values of the filtered signals (1D array of size `num_signals`).
        """
        if self.zi is None:
            # Initialize the filter state for each signal (column)
            self.zi = signal.lfilter_zi(self.b, self.a).reshape(-1, 1) * window[0:1, :]

        # Apply the Butterworth filter
        filtered_window, self.zi = signal.lfilter(self.b, self.a, window, axis=0, zi=self.zi)
        
        return filtered_window[self.middle_index, :]
    


class DataFilter:
    def __init__(self, window_size, filter_type, gui_commands_topic):
        rospy.loginfo("Starting GUI...")
        self.window_size = window_size
        self.window = np.zeros((window_size, 10))
        self.num_valid_points = 0  # Track valid points added
        self.full = False

        self.filter_type = filter_type
        self.valid_filters = ["mean", "median", "biological", "butterworth"]
        gui_sub = rospy.Subscriber(gui_commands_topic, String, self.gui_callback)

        self.butterworth = ButterworthFilter(order=1, cutoff=0.7, sampling_rate=5.3, window_size=self.window_size)
        self.biological = BiologicalMotionFilter(window_size=self.window_size, regularization=1.0)

    def get_filtered_angles(self, data_point: np.ndarray):
        # Shift rows up, removing the oldest data point
        print("Data Point: ", data_point)
        self.window[:-1] = self.window[1:]
        
        # Add the new data point as the last row
        self.window[-1] = data_point
        print("Window: ", self.window)
        # Update the number of valid points in the window
        if self.num_valid_points < self.window_size:
            self.num_valid_points += 1

        # Mark window as full if enough points are added
        if self.num_valid_points >= self.window_size:
            self.full = True

        # Apply the filter only if the window is full
        if self.full:
            if self.filter_type == "mean":
                return np.mean(self.window, axis=0)  # Mean across rows for each column
            elif self.filter_type == "median":
                return np.median(self.window, axis=0)  # Median across rows for each column
            elif self.filter_type == "butterworth":
                return self.butterworth.filter(self.window)
            elif self.filter_type == "biological":
                return self.biological.filter(self.window)
            else:
                raise ValueError(f"Filter type '{self.filter_type}' is not supported.")
        else:
            return None  # Not enough data to apply the filter yet


    def gui_callback(self, msg):
        if msg.data.startswith("FILTER"):
            self.set_filter_type(msg.data[6:])
        
    def set_filter_type(self, filter_type):
        """
        Set the filter type for the DataFilter instance.
        
        Args:
        - filter_type (str): The new filter type to apply ("mean", "median", etc.).
        
        Raises:
        - ValueError: If the provided filter type is not recognized.
        """
        # Add more valid filter types as needed
        if filter_type not in self.valid_filters:
            raise ValueError(f"Invalid filter type '{filter_type}'. Valid options are: {self.valid_filters}")
        self.filter_type = filter_type
        rospy.loginfo(f"Set filter to {filter_type}.")
            
    

