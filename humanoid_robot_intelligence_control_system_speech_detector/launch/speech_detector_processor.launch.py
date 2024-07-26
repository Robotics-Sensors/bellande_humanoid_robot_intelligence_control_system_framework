# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def ros1_launch_description():
    # Get command-line arguments
    args = sys.argv[1:]
    
    # Construct the ROS 1 launch command
    roslaunch_command = ["roslaunch", "humanoid_robot_intelligence_control_system_speech_detector", "speech_detector_processor.launch"] + args
    
    roslaunch_command.extend([
        "usb_cam", "usb_cam_node", "name:=camera",
        "video_device:=/dev/video0",
        "image_width:=640",
        "image_height:=480",
        "pixel_format:=yuyv",
        "camera_frame_id:=usb_cam",
        "io_method:=mmap"
    ])
    
    roslaunch_command.extend([
        "ros_web_api_bellande_2d_computer_vision", "bellande_2d_computer_vision_speech_detection.py", "name:=speech_detection_node"
    ])
    
    roslaunch_command.extend([
        "humanoid_robot_intelligence_control_system_ball_detector", "speech_detection_processor.py", "name:=speech_detection_processor_node"
    ])
    
    roslaunch_command.extend([
        "rviz", "rviz", "name:=rviz",
        "args:=-d $(find ros_web_api_bellande_2d_computer_vision)/rviz/visualization.rviz"
    ])
    
    # Execute the launch command
    subprocess.call(roslaunch_command)

def ros2_launch_description():
    nodes_to_launch = []
    
    nodes_to_launch.append(Node(
        package='usb_cam',
        executable='usb_cam_node',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap'
        }]
    ))
    
    nodes_to_launch.append(Node(
        package='ros_web_api_bellande_2d_computer_vision',
        executable='bellande_2d_computer_vision_speech_detection.py',
        name='speech_detection_node',
        output='screen',
        remappings=[('camera/image_raw', '/usb_cam/image_raw')]
    ))
    
    nodes_to_launch.append(Node(
        package='humanoid_robot_intelligence_control_system_speech_detector',
        executable='speech_detection_processor.py',
        name='speech_detection_processor_node',
        output='screen',
        parameters=[{'yaml_path': '$(find ros_web_api_bellande_2d_computer_vision)/yaml/speech_detection_params.yaml'}]
    ))
    
    nodes_to_launch.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', '$(find ros_web_api_bellande_2d_computer_vision)/rviz/visualization.rviz']
    ))
    
    return LaunchDescription(nodes_to_launch)

if __name__ == "__main__":
    ros_version = os.getenv("ROS_VERSION")
    if ros_version == "1":
        ros1_launch_description()
    elif ros_version == "2":
        ros2_launch_description()
    else:
        print("Unsupported ROS version. Please set the ROS_VERSION environment variable to '1' for ROS 1 or '2' for ROS 2.")
        sys.exit(1)
