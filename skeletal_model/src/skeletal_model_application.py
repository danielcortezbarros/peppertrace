#!/usr/bin/env python3
""" skeletal_model_application.py Skeletal tracking and angle retargeting to Pepper robot
    
    This component subscribes to the color and depth image feed of the camera ROS driver 
    and estimates the joint positions on the human body with mediapipe and the depth info. 
    It then calculates the retargeted angles to the Pepper robot. After a filter is applied,
    the angles are published to the robot arm controllers.
    ...
    Libraries
    Python Libraries
    - sys, os, cv2, json, mediapipe, numpy, math, time, threading
    ROS Libraries
    - rospkg, rospy, cv_bridge, sensor_msgs, message_filters, trajectory_msgs
    ...
    Parameters
    Command-line Parameters
    ...
    Configuration File Parameters

    Parameter | Type
    camera_intrinsics   | list
    image_width         | int
    image_height        | int
    color_image_topic   | str
    depth_image_topic   | str
    left_arm_cmd_topic  | str
    right_arm_cmd_topic | str
    ...
    Subscribed Topics and Message Types
    
    Topic | Type
    /camera/color/image_raw                  | sensor_msgs/Image
    /camera/aligned_depth_to_color/image_raw | sensor_msgs/Image
    /gui/commands                            | std_msgs/String
    ...
    Published Topics and Message Types

    /pepper_dcm/LeftArm_controller/command   | trajectory_msgs/JointTrajectory
    /pepper_dcm/RightArm_controller/command  | trajectory_msgs/JointTrajectory
    /mediapipe/image_feed                    | sensor_msgs/Image
    ...
    Input Data Files
    skeletal_model_input.dat
    ...
    Output Data Files
    skeletal_model_output.dat
    ...
    Configuration Files
    skeletal_model_configuration.json
    ...
    Example Instantiation of the Module
    rosrun skeletal_model skeletal_model_application.py 
    ...
    Author: Daniel Barros, CMU-Africa
    Email: daniel.barros@tum.de
    Date: 21 November, 2024
    Version: v1.0
"""

import sys
import os
import rospkg 

#Add implementation modules to python path
package_path = rospkg.RosPack().get_path('programming_from_demonstration')
src_path = os.path.join(package_path, 'skeletal_model', 'src')
sys.path.insert(0, src_path)

import rospy
import json
from skeletal_model_estimation_implementation import SkeletalModelEstimation

def main():
    rospy.init_node('skeletal_model', anonymous=False)

    # Construct the path to the config file relative to the package path
    config_file_path = os.path.join(package_path, 'skeletal_model', 'config', 'skeletal_model_configuration.json')

    # Load the JSON config file
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)

    # Instantiate skeletal model class with config parameters
    skeletal_model = SkeletalModelEstimation(camera_intrinsics=config["camera_intrinsics"],
                                   image_width=config["image_width"],
                                   image_height=config["image_height"],
                                   color_image_topic=config["color_image_topic"],
                                   depth_image_topic=config["depth_image_topic"],
                                   left_arm_command_topic=config["left_arm_command_topic"],
                                   right_arm_command_topic=config["right_arm_command_topic"], 
                                   gui_commands_topic=config["gui_commands_topic"],
                                   skeletal_model_feed_topic=config["skeletal_model_feed_topic"]
                                   )

    # Run ROS loop to get Pepper angles from images
    try:
        skeletal_model.run()  
    except rospy.ROSInterruptException:
        rospy.loginfo("Skeletal model was stopped.")

if __name__ == '__main__':
    main()
