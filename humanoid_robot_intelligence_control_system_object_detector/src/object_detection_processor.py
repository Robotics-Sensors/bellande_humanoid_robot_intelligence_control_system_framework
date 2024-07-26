#!/usr/bin/env python3

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

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import yaml

class ObjectDetectionProcessor:
    def __init__(self):
        rospy.init_node('object_detection_processor')
        self.bridge = CvBridge()
        self.enable = True
        self.new_image_flag = False
        self.load_params()
        self.setup_ros()

    def load_params(self):
        param_path = rospy.get_param('~yaml_path', '')
        if param_path:
            with open(param_path, 'r') as file:
                self.params = yaml.safe_load(file)
        else:
            self.set_default_params()

    def set_default_params(self):
        self.params = {
            'debug': False,
            'ellipse_size': 2,
            # Add other default parameters as needed
        }

    def setup_ros(self):
        self.image_pub = rospy.Publisher('image_out', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
        
        rospy.Subscriber('enable', Bool, self.enable_callback)
        rospy.Subscriber('image_in', Image, self.image_callback)
        rospy.Subscriber('cameraInfo_in', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('object_detection_result', String, self.object_detection_callback)

    def enable_callback(self, msg):
        self.enable = msg.data

    def image_callback(self, msg):
        if not self.enable:
            return
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.new_image_flag = True
        self.image_header = msg.header

    def camera_info_callback(self, msg):
        if not self.enable:
            return
        self.camera_info_msg = msg

    def object_detection_callback(self, msg):
        if not self.enable or not hasattr(self, 'cv_image'):
            return
        
        objects = eval(msg.data)  # Assuming the data is a string representation of a list
        self.process_detected_objects(objects)

    def process_detected_objects(self, objects):
        output_image = self.cv_image.copy()
        for obj in objects:
            x, y, w, h = obj['bbox']
            cv2.rectangle(output_image, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
            cv2.putText(output_image, f"{obj['label']}: {obj['confidence']:.2f}", 
                        (int(x), int(y-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        self.publish_image(output_image)

    def publish_image(self, image):
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header = self.image_header
        self.image_pub.publish(img_msg)
        if hasattr(self, 'camera_info_msg'):
            self.camera_info_pub.publish(self.camera_info_msg)

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            if self.new_image_flag:
                # The processing is done in object_detection_callback
                self.new_image_flag = False
            rate.sleep()

if __name__ == '__main__':
    try:
        processor = ObjectDetectionProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
