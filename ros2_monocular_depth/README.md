The repository contains ROS2 package for [MiDAS : Robust Monocular Depth Estimation](https://github.com/intel-isl/MiDaS).

## Package outputs:
- Depth Estimated Image

## Requirements
- ROS2 Eloquent <br/>
    - [Docker Support](https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/docker)<br/>
    - [Install Scripts](https://github.com/NVIDIA-AI-IOT/ros2_jetson/tree/main/scripts)<br/>

- [torch2trt](https://github.com/NVIDIA-AI-IOT/torch2trt)
    - [Install torch2trt](https://github.com/NVIDIA-AI-IOT/torch2trt#option-2---with-plugins-experimental)

## Build
- Clone repository under ros2 workspace <br/>
```
$ cd ros2_ws/src/
$ git clone <repo name>
```

- Install requirements using `rosdep` <br/>
``` $ rosdep install --from-paths src --ignore-src --rosdistro eloquent -y```

- torch2trt related model file updates <br/>
    - Clone the MiDaS repository<br/>
    ```
    $ cd ros2_ws/src/ros2_monocular_depth/ros2_monocular_depth
    $ git clone https://github.com/intel-isl/MiDaS
    ```
    - To resolve a torch2trt tracing error, modify the following line https://github.com/intel-isl/MiDaS/blob/f275ca1c6f9af17aabe6f1e024b2084d7b84abb0/midas/blocks.py#L117 to read<br/>
    ```
    x = nn.functional.interpolate(
    x, scale_factor=self.scale_factor, mode=self.mode, align_corners=False
    )
    ```
    - Download the MiDaS small model weights into `model_dir`. Name them midas_trt.pth.<br/>

- Build and install `ros2_monocular_depth` package <br/>

``` 
$ colcon build
$ source install/local_setup.sh
```

## Run

- Change Power Mode for Jetson
``` sudo nvpmodel -m2 ``` (for Jetson Xavier NX) <br/>
``` sudo nvpmodel -m0 ``` (for Jetson Xavier and Jetson Nano) <br/>
- Input Images are captured using ```image_tools``` package <br/>
``` ros2 run image_tools cam2image ```
- Run `ros2_monocular_depth` node <br/>
```
$ ros2 run ros2_monocular_depth depth_estimation --ros-args -p model_dir:='<absolute-path-to-base_dir>'
```

## Other related work <br/>

[ROS2 Jetson AI Support](https://nvidia-ai-iot.github.io/ros2_jetson/)


