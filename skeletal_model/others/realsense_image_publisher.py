#!/usr/bin/env python3

#This script publishes aligned color and depth images from the Intel RealSense camera to a ROS topic at a specific rate

import rospy
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import datetime
import os

class RealSensePublisher:
    def __init__(self, folder_path):
        # Initialize the ROS node
        rospy.init_node('realsense_image_publisher', anonymous=True)

        # Publishers for color and aligned depth images
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)

        self.rate = 1.0

        # Create a CvBridge for converting between OpenCV images and ROS images
        self.bridge = CvBridge()

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Enable color and depth streams
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Try to start the pipeline with the configuration
        try:
            self.pipeline_profile = self.pipeline.start(config)
            rospy.loginfo("RealSense pipeline started successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to start RealSense pipeline: {e}")
            raise

        # Align depth to color
        self.align = rs.align(rs.stream.color)

        # For tracking the time between publishing images
        self.last_capture_time = rospy.Time.now()

        self.folder_path = folder_path
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)


    def capture_and_publish(self):
        while not rospy.is_shutdown():
            try:
                # Wait for frames and align the depth to the color frame
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)

                # Get the aligned depth and color frames
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()

                if not color_frame or not depth_frame:
                    rospy.logwarn("No frames received from RealSense camera.")
                    continue

                # Convert RealSense frames to numpy arrays (OpenCV compatible)
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())

                # print("Color Shape: ", color_image.shape)
                # print("Depth Shape: ", depth_image.shape)

                # Display the images in OpenCV windows
                cv2.imshow('Color Image', color_image)
                cv2.imshow('Aligned Depth Image', depth_image)

                # Check if 5 seconds have passed since the last capture
                current_time = rospy.Time.now()
                if (current_time - self.last_capture_time).to_sec() >= self.rate:
                    rospy.loginfo(f"{self.rate} seconds passed. Capturing and publishing images...")

                    # Convert OpenCV images to ROS Image messages
                    color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")

                    # Publish the images
                    self.color_pub.publish(color_msg)
                    self.depth_pub.publish(depth_msg)

                    rospy.loginfo("Images published successfully")

                    # Update the last capture time
                    self.last_capture_time = current_time

                # Allow OpenCV to refresh windows
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                rospy.logerr(f"Error during frame processing: {e}")
                continue

        # Stop the RealSense pipeline
        self.pipeline.stop()

    def capture_and_save(self):
        try:
            rospy.loginfo("Waiting 5 seconds before capturing images...")
            rospy.sleep(5)

            # Wait for frames and align the depth to the color frame
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            # Get the aligned depth and color frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                rospy.logwarn("No frames received from RealSense camera.")
                return

            # Convert RealSense frames to numpy arrays (OpenCV compatible)
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Generate filenames with timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            color_filename = os.path.join(self.folder_path, f"color_{timestamp}.png")
            depth_filename = os.path.join(self.folder_path, f"depth_{timestamp}.png")

            # Save the images
            cv2.imwrite(color_filename, color_image)
            cv2.imwrite(depth_filename, depth_image)

            rospy.loginfo(f"Saved color image: {color_filename}")
            rospy.loginfo(f"Saved depth image: {depth_filename}")

        except Exception as e:
            rospy.logerr(f"Error during frame processing: {e}")
        finally:
            # Stop the RealSense pipeline
            self.pipeline.stop()

if __name__ == "__main__":
    path='/root/workspace/pepper_rob_ws/images'
    try:
        node = RealSensePublisher(path)
        node.capture_and_save()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
