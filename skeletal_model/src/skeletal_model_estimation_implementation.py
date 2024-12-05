""" skeletal_model_implementation.py Implements skeletal tracking from images to Pepper angles

    Author:Daniel Barros
    Date: November 21, 2024
    Version: v1.0

    Copyright (C) 2023 CSSR4Africa Consortium

    This project is funded by the African Engineering and Technology Network (Afretec
    Inclusive Digital Transformation Research Grant Programme.
    
    Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import mediapipe as mp
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import message_filters
import time 
import threading
from programming_from_demonstration.skeletal_model.src.skeletal_model_retargeting_implementation import HumanToHumanoidRetargeting
from programming_from_demonstration.skeletal_model.src.skeletal_model_filters_implementation import DataFilter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

# Decorator to measure the execution time
def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Execution time of {func.__name__}: {execution_time:.4f} seconds")
        return result
    return wrapper


def pixel_to_3d_camera_coords(x, y, depth, K_matrix):
    """
    Convert relative 2D image coordinates (normalized between 0 and 1) and depth to 3D coordinates in world coordinates in the camera frame of reference

    Args:
        x (float): Normalized x-coordinate (between 0 and 1).
        y (float): Normalized y-coordinate (between 0 and 1).
        image_shape (tuple): Shape of the image (height, width).
        K (np.ndarray): 3x3 camera intrinsic matrix.
        depth (float): Depth value (in meters).

    Returns:
        np.ndarray: 3D position in the camera's reference frame (X, Y, Z).
    """

    # Pixel coordinates in homogeneous form [u, v, 1]
    point_2d_hom = np.array([x, y, 1])

    # Inverse of the intrinsic matrix
    K_matrix_inv = np.linalg.inv(K_matrix)

    # Apply the formula: 3D position = Z * K^-1 * pixel_coords_homogeneous
    point_3d = depth * (K_matrix_inv @ point_2d_hom)

    # Return the X, Y, Z coordinates
    return point_3d


class SkeletalModelEstimation:
    def __init__(self, camera_intrinsics, 
                 image_width, 
                 image_height, 
                 color_image_topic, 
                 depth_image_topic, 
                 left_arm_command_topic, 
                 right_arm_command_topic,
                 gui_commands_topic,
                 skeletal_model_feed_topic):

        self.intrinsics = camera_intrinsics
        self.image_width = image_width
        self.image_height = image_height
        self.bridge = CvBridge()  # Initialize the CvBridge class
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

        self.image_lock = threading.Lock()
        self.latest_image = None
        self.shutdown_flag = False
        self.filter = DataFilter(20, "mean")

        # Subscribe to both the color and depth topics
        color_sub = message_filters.Subscriber(color_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)
        gui_sub = rospy.Subscriber(gui_commands_topic, String, self.gui_callback)
        self.left_arm_command_pub = rospy.Publisher(left_arm_command_topic, JointTrajectory, queue_size=10)
        self.right_arm_command_pub = rospy.Publisher(right_arm_command_topic, JointTrajectory, queue_size=10)
        self.image_pub = rospy.Publisher(skeletal_model_feed_topic, Image, queue_size=1)

        self.left_arm_joint_names = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"]
        self.right_arm_joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]


        # Synchronize the image and depth topics
        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.04)
        ts.registerCallback(self.image_callback)

        self.lock = threading.Lock() 

    def gui_callback(self, msg):
        if msg.data.startswith("FILTER"):
            self.filter.set_filter_type(msg.data)
 

    def image_callback(self, image_data, depth_data):
        # Acquire the lock to ensure only one callback is processed at a time
        rospy.loginfo("Received synchronized images")
        if not self.lock.acquire(blocking=False):
            print('Skipping')
            return

        try:
            # Convert ROS Image message to OpenCV image for both color and depth
            bgr_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
            cv2.imshow('image', bgr_image)
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")  # Depth is usually 16-bit unsigned

            # Convert to RGB for MediaPipe processing
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            rgb_image.flags.writeable = False
            
            landmarks=self.get_landmarks(bgr_image)

            if landmarks is not None:
                angles = self.get_pepper_angles(landmarks, depth_image)
            else:
                rospy.loginfo("Landmarks could not be calculated")
                return
                

            if angles is not None:
                filtered_angles = self.filter.get_filtered_angles(angles)
                self.publish_angles(filtered_angles)
            else:
                rospy.loginfo("No landmarks for all joints, skipped angle calculation")
                return

        except CvBridgeError as e:
            rospy.logerr(f"CvBridgeError: {e}")
        finally:
            # Release the lock when done processing
            self.lock.release()

    def get_landmarks(self, bgr_image):
        # Convert to RGB for MediaPipe processing
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        rgb_image.flags.writeable = False
        # Perform pose detection with MediaPipe
        results = self.mp_pose.process(rgb_image)
        # Get pose landmarks for left shoulder, elbow, wrist
        if results.pose_landmarks:
            # Overlay skeleton on the original image and store it for display
            self.mp_drawing.draw_landmarks(bgr_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
        
            with self.image_lock:
                self.latest_image = bgr_image.copy()

            return results.pose_landmarks.landmark
        
        else:
            return None
        
    def get_3d_coordinates(self,landmark, depth_image):
        # Get the 2D pixel coordinates
        x = int(landmark.x * self.image_width)
        y = int(landmark.y * self.image_height)

        # Ensure the x and y coordinates are within the depth image bounds
        if x < 0 or x >= depth_image.shape[1] or y < 0 or y >= depth_image.shape[0]:
            return None  # Out of bounds

        # Get the depth value (z-coordinate)
        z = depth_image[y, x]

        # Ensure valid depth (non-zero and non-NaN)
        if z == 0 or np.isnan(z):
            return None

        # Convert 2D pixel coordinates and depth to 3D camera coordinates
        return pixel_to_3d_camera_coords(x, y, z, self.intrinsics)
    

    def get_pepper_angles(self, landmarks, depth_image):
        """
        Calculate the shoulder pitch, shoulder roll, elbow roll, elbow yaw, and wrist yaw angles for both arms.

        Args:
            joints (dict): Dictionary of joint 3D coordinates.
            Expects keys 'Neck', 'LShoulder', 'LElbow', 'LWrist', 'RShoulder', 'RElbow', 'RWrist', 'MidHip'.

        Returns:
            dict: A dictionary containing angles for both arms (left and right).
        """

        
        def mid_point(P1, P2):
            return [
                (P1[0] + P2[0]) / 2,
                (P1[1] + P2[1]) / 2,
                (P1[2] + P2[2]) / 2
            ]

        # First, calculate the 3D coordinates for all keypoints from MediaPipe and depth image
        LShoulder = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER.value], depth_image)
        RShoulder = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER.value], depth_image)
        LElbow = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_ELBOW.value], depth_image)
        RElbow = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW.value], depth_image)
        LWrist = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_WRIST.value], depth_image)
        RWrist = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_WRIST.value], depth_image)
        LHip = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.LEFT_HIP.value], depth_image)
        RHip = self.get_3d_coordinates(landmarks[mp.solutions.pose.PoseLandmark.RIGHT_HIP.value], depth_image)

        
        # Check if any 3D coordinates are None (invalid) and skip this frame if they are
        if any(coord is None for coord in [LShoulder, RShoulder, LElbow, RElbow, LWrist, RWrist, LHip, RHip]):
            rospy.logwarn("One or more keypoints are out of bounds or invalid")
            return None

        # Calculate Neck and MidHip
        Neck = mid_point(LShoulder, RShoulder)
        MidHip = mid_point(LHip, RHip)

        # Now construct the wp_dict
        wp_dict = {
            '1': Neck,         # Neck
            '2': RShoulder,    # Right Shoulder
            '3': RElbow,       # Right Elbow
            '4': RWrist,       # Right Wrist
            '5': LShoulder,    # Left Shoulder
            '6': LElbow,       # Left Elbow
            '7': LWrist,       # Left Wrist
            '8': MidHip        # MidHip
        }

        # Finally, call the get_angles function
        calc_util=HumanToHumanoidRetargeting()
        angles = calc_util.get_angles(wp_dict)
        angles = {
            'LShoulderPitch': angles[0],
            'LShoulderRoll': angles[1],
            'LElbowRoll': angles[2],
            'LElbowYaw': angles[3],
            'LWristYaw': 0,
            'RShoulderPitch': angles[4],
            'RShoulderRoll': angles[5],
            'RElbowRoll': angles[6],
            'RElbowYaw': angles[7],
            'RWristYaw': 0
        }

        print(angles)

        return angles
    

    def publish_angles(self, angles):
        # Publish left arm joint angles
        left_traj_msg = JointTrajectory()
        left_traj_msg.joint_names = self.left_arm_joint_names

        # Create a JointTrajectoryPoint for the left arm
        left_point = JointTrajectoryPoint()
        left_point.positions = [angles[joint] for joint in self.left_arm_joint_names]
        left_point.time_from_start = rospy.Duration(0.033)  # Set time to 33 ms (30 FPS)

        left_traj_msg.points = [left_point]  # Add the point to the trajectory

        # Publish right arm joint angles
        right_traj_msg = JointTrajectory()
        right_traj_msg.joint_names = self.right_arm_joint_names

        # Create a JointTrajectoryPoint for the right arm
        right_point = JointTrajectoryPoint()
        right_point.positions = [angles[joint] for joint in self.right_arm_joint_names]
        right_point.time_from_start = rospy.Duration(0.033)  # Set time to 33 ms (30 FPS)

        right_traj_msg.points = [right_point]  # Add the point to the trajectory

        future_time = rospy.Time.now() + rospy.Duration(0.05)
        left_traj_msg.header.stamp = future_time
        right_traj_msg.header.stamp = future_time

        # Publish the messages to the left and right arm controllers
        self.left_arm_command_pub.publish(left_traj_msg)
        self.right_arm_command_pub.publish(right_traj_msg)

        # self.bag.write(self.left_arm_command_topic, left_traj_msg)
        # self.bag.write(self.right_arm_command_topic, right_traj_msg)

    def run(self):
        """Main loop to continuously display the image feed."""
        while not rospy.is_shutdown() and not self.shutdown_flag:
            with self.image_lock:
                if self.latest_image is not None:
          
                    # Publish the image
                    compressed_image = cv2.resize(self.latest_image, (640, 360))
                    ros_image = self.bridge.cv2_to_imgmsg(compressed_image, encoding="bgr8")
                    
                    self.image_pub.publish(ros_image)
                    # cv2.imwrite('/root/workspace/pepper_rob_ws/images/debug_output.png', self.latest_image)

        # Clean up the windows after the loop exits
        cv2.destroyAllWindows()

