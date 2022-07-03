# The MIT License (MIT)
#
# Copyright (c) 2021, NVIDIA CORPORATION
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# ROS2 related
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg

from rclpy.duration import Duration
from ros2_monocular_depth.utils import convert_midas, load_model, preprocess
import numpy as np
import cv2
import os

class TRTMiDaS(Node):
    def __init__(self):
        super().__init__('trt_midas')
        self.trt_midas_model = None
        self.image = None
        self.depth_image = None
        # ROS2 parameters
        self.declare_parameter('model_dir', '/home/ak-nv/trt_pose/tasks/human_pose')
        self.model_dir = self.get_parameter('model_dir')._value
        # Image subscriber from cam2image
        self.subscriber_ = self.create_subscription(ImageMsg, 'image', self.read_cam_callback, 10)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 10)
        self.trt_output_model_path = os.path.join(self.model_dir, 'midas_trt.pth')
        self.weight_path = os.path.join(self.model_dir, 'model_small.pt')

    def start(self):
        if not os.path.isfile(self.trt_output_model_path):
            self.get_logger().info("Converting MiDaS Model to TensorRT")
            convert_midas(model_path=self.weight_path, trt_model_path=self.trt_output_model_path, in_width=256, in_height=256)
        self.get_logger().info("Loading MiDaS TensorRT model ")
        self.trt_midas_model = load_model(trt_model_path=self.trt_output_model_path, in_height=256, in_width=256)
        self.get_logger().info("Model weights loaded...\n Waiting for images...\n")


    def execute(self):
        data = preprocess(image=self.image, width=256, height=256)
        output = self.trt_midas_model(data)
        output = output[0].detach().cpu().numpy()
        output = (output - output.min()) / (output.max() - output.min())
        output = (output * 255).astype(np.uint8)
        return output

    # Subscribe and Publish to image topic
    def read_cam_callback(self, msg):
        img = np.asarray(msg.data)
        self.image = np.reshape(img, (msg.height, msg.width, 3))
        self.get_logger().info("Executing MiDaS: depth estimation using TensorRT (torch2trt)...\n")

        cv2.imshow("Input Image", self.image)
        self.depth_image = self.execute()

        #image_msg = self.image_np_to_image_msg(self.depth_image)
        #self.image_pub.publish(image_msg)
        # if self.show_image_param:
        cv2.imshow('Depth Image', self.depth_image)
        cv2.waitKey(1)

