from collections import deque
import rospy
import numpy as np


# Define static functions for filters
def mean_filter(window):
    """Compute the mean of the window."""
    return sum(window) / len(window)

def median_filter(window):
    """Compute the median of the window."""
    sorted_window = sorted(window)
    mid = len(sorted_window) // 2
    if len(sorted_window) % 2 == 0:
        return (sorted_window[mid - 1] + sorted_window[mid]) / 2
    else:
        return sorted_window[mid]
    
def butterworth(window):
    print("Banana")

def biological(window):
    print("Banana")
    
filters = {
    "mean": mean_filter,
    "median": median_filter,
    "butterworth": butterworth,
    "biological": biological
}

class DataFilter:
    def __init__(self, window_size, filter_type="mean"):
        """
        Initialize the DataFilter class.
        
        Args:
        - window_size (int): The size of the window for filtering.
        - filter_type (str): The type of filter to apply ("mean", "median", etc.).
        """
        self.window_size = window_size
        self.window = deque(maxlen=window_size)
        self.filter_type = filter_type
        self.valid_filters = filters.keys()

    def _apply_filter(self):
        """Apply the selected filter to the current window."""

        if self.filter_type == "mean":
            return mean_filter(self.window)
        elif self.filter_type == "median":
            return median_filter(self.window)
        else:
            raise ValueError(f"Unknown filter type: {self.filter_type}")

    def get_filtered_angles(self, data_point:np.array):
        """
        Add a new data point to the window, remove the oldest if necessary,
        and return the filtered value.
        
        Args:
        - data_point: The new data point to add.
        
        Returns:
        - The result of the filter applied to the window.
        """
        self.window.append(data_point)
        if len(self.window) == self.window_size:
            return self._apply_filter()
        else:
            return None  # Not enough data to apply the filter yet.
        
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
            
    

# Example Usage
data_filter = DataFilter(window_size=5, filter_type="mean")

# # Simulate incoming data
# for i in range(10):
#     filtered_value = data_filter.add_data_point(i)
#     if filtered_value is not None:
#         print(f"New filtered value: {filtered_value}")
