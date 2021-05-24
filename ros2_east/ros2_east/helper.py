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

import numpy as np
import cv2
import os
import sys
import torch
sys.path.append('/home/ak-nv/ros2_ws/src/ros2_east/ros2_east/EAST')
from model import * # from EAST for EAST model class
from detect import * # from EAST for detect, plot_boxes
from torch2trt import torch2trt
from torch2trt import TRTModule

from PIL import Image


_device = torch.device('cuda') # don't change

class TRTEast(Node):
    def __init__(self):
        super().__init__('trt_east')
        self.trt_east_model = None
        self.image = None
        self.annotated_image = None
        self.width = 256
        self.height = 256
        # ROS2 parameters
        self.declare_parameter('show_image', True) # Show image in cv2.imshow
        self.show_image_param = self.get_parameter('show_image')._value

        self.model_path = os.getenv("HOME")+'/pths/'
        self.trt_model_file = os.path.join(self.model_path, 'east_trt.pth')
        # Image subscriber from cam2image
        self.subscriber_ = self.create_subscription(ImageMsg, 'image', self.read_cam_callback, 10)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 10)

    def start(self):
        if not os.path.isfile(self.trt_model_file):
            self.get_logger().info("Loading East Model and Optimizing to TensorRT")
            self.convert_east()

        self.get_logger().info("Loading TRT optimized East Model")
        self.trt_east_model = self.load_model()
        self.get_logger().info("Model weights loaded...\n Waiting for images...\n")

    def convert_east(self):
        model = EAST()
        model = model.cuda().eval()
        model.load_state_dict(torch.load(os.getenv("HOME")+'/pths/east_vgg16.pth'))
        data = torch.randn(1, 3, self.width, self.height).cuda()
        model_trt = torch2trt(model, [data], fp16_mode=True)
        torch.save(model_trt.state_dict(), self.trt_model_file)

    def load_model(self):
        model_trt = TRTModule()
        model_trt.load_state_dict(torch.load(self.trt_model_file))
        return model_trt

    def execute(self):
        img = Image.fromarray(self.image).resize((256,256))
        im_w , im_h = img.size
        boxes = detect(img, self.trt_east_model, _device)
        self.annotated_image = plot_boxes(img, boxes)

    def read_cam_callback(self, msg):
        img = np.asarray(msg.data)
        self.image = np.reshape(img, (msg.height, msg.width, 3))
        #self.get_logger().info("Executing EAST: a SCENE text detector using TensorRT (torch2trt)...\n")

        self.execute()
        image_msg = self.image_np_to_image_msg(self.annotated_image)
        self.image_pub.publish(image_msg)
        if self.show_image_param:
            cv2.imshow('EAST Text Detection ROS', np.array(self.annotated_image))
            cv2.waitKey(1)

    # Borrowed from OpenPose-ROS repo
    def image_np_to_image_msg(self, image_np):
        image_msg = ImageMsg()
        image_msg.width, image_msg.height = image_np.size
        image_msg.encoding = 'bgr8'
        image_msg.data = image_np.tobytes()
        image_msg.step = len(image_msg.data) // image_msg.height
        image_msg.header.frame_id = 'map'
        return image_msg
