# ros2_torch2trt_examples

The repository contains ros2 nodes for ```torch2trt``` examples.
[```torch2trt```](https://github.com/NVIDIA-AI-IOT/torch2trt) is a PyTorch to TensorRT converter which utilizes the TensorRT Python API.

Following examples are currently supported in ROS2:<br/>
	- [MiDAS: Robust Monocular Depth Estimation](https://github.com/intel-isl/MiDaS)<br/>
	- [EAST: An Efficient and Accurate Scene Text Detector](https://github.com/SakuraRiven/EAST)<br/>

## Requirements
- ROS2 Eloquent <br/>
    - [Docker Support](https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/docker)<br/>
    - [Install Scripts](https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/scripts)<br/>

- [torch2trt](https://github.com/NVIDIA-AI-IOT/torch2trt)
    - [Install torch2trt](https://github.com/NVIDIA-AI-IOT/torch2trt#option-2---with-plugins-experimental)

Please use ReadMe for each package for built, and run instructions.