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

import sys
import cv2
import torch
from torchvision.transforms import Compose

sys.path.append('/home/ak-nv/ros2_ws/src/ros2_monocular_depth/ros2_monocular_depth/MiDaS')
from torch2trt import torch2trt, trt
from torch2trt import TRTModule
import PIL

from .MiDaS.midas.midas_net_custom import MidasNet_small
from torchvision.transforms import Compose
from .MiDaS.midas.midas_net import MidasNet
from .MiDaS.midas.midas_net_custom import MidasNet_small
from .MiDaS.midas.transforms import Resize, NormalizeImage, PrepareForNet


def convert_midas(model_path, trt_model_path, in_width, in_height):
    # Requires modifying some of the code in MiDaS
    model = MidasNet_small(model_path, features=64, backbone="efficientnet_lite3", exportable=True,
                           non_negative=True, blocks={'expand': True})
    model = model.cuda().eval()
    data = torch.rand(1, 3, in_height, in_width).cuda()
    model_trt = torch2trt(model, [data])

    torch.save(model_trt.state_dict(), trt_model_path)

def load_model(trt_model_path, in_width, in_height):
    model_trt = TRTModule()
    model_trt.load_state_dict(torch.load(trt_model_path))
    return model_trt

# Pre-process image message received from cam2image
def preprocess(image, width, height):
    image = cv2.resize(image, (width, height))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) / 255.0
    transform = Compose(
        [
            Resize(
                width,
                height,
                resize_target=None,
                keep_aspect_ratio=False,
                # we use False because we must guarantee 256x256 shape since this is what we optimized engine using
                ensure_multiple_of=32,
                resize_method="upper_bound",
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            PrepareForNet(),
        ]
    )

    data = transform({'image': image})['image']
    data = torch.from_numpy(data)[None, ...].cuda()
    return data



