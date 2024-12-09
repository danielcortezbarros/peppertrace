# import rospy
import numpy as np
from std_msgs.msg import String


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
    def __init__(self, window_size, filter_type, gui_commands_topic):
        """
        Initialize the DataFilter class.
        
        Args:
        - window_size (int): The size of the window for filtering.
        - filter_type (str): The type of filter to apply ("mean", "median", etc.).
        """
        self.window_size = window_size
        self.window = np.zeros((window_size, 10))
        self.current_index = 0
        self.full = False

        self.filter_type = filter_type
        self.valid_filters = filters.keys()
        gui_sub = rospy.Subscriber(gui_commands_topic, String, self.gui_callback)

        

    def get_filtered_angles(self, data_point: np.ndarray):
        """
        Add a new data point to the window, overwrite the oldest if necessary,
        and return the filtered value for each joint.
        
        Args:
        - data_point: The new data point to add (1D NumPy array of angles).
        
        Returns:
        - The result of the filter applied to the window (1D NumPy array).
        """
        # Add the new data point to the current index
        self.window[self.current_index] = data_point

        # Update the index and check if the window is full
        self.current_index = (self.current_index + 1) % self.window_size
        if self.current_index == 0:
            self.full = True  # The window is now fully populated

        # Apply the filter only if the window is full
        if self.full:
            if self.filter_type == "mean":
                return np.mean(self.window, axis=0)
            elif self.filter_type == "median":
                return np.median(self.window, axis=0)
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
            
    

# Initialize the filter with a window size of 5
# data_filter = DataFilter(window_size=5, filter_type="mean", gui_commands_topic="/gui/commands")

# # Simulate incoming angles
# for i in range(10):
#     data_point = np.random.rand(10)  # 10 random angles
#     filtered = data_filter.get_filtered_angles(data_point)
#     if filtered is not None:
#         print(f"Filtered angles (step {i}): {filtered}")
#     else:
#         print(f"Not enough data to filter (step {i})")
