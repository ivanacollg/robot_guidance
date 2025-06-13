# Robot Guidance 
- ROS2 Jazzy 
- Python 3

## Install 

Use of a virtual environment is incouraged: 
```
sudo apt install python3.12-venv
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
```

### Install Dependencies

System wide intalls:
```
sudo apt install ros-jazzy-usb-cam
sudo apt install ros-jazzy-apriltag-msgs
sudo apt install ros-jazzy-apriltag-ros
sudo apt install ros-jazzy-image-proc
sudo apt install ros-jazzy-nav2-lifecycle-manager
sudo apt install ros-jazzy-v4l2-camera
sudo apt install ros-jazzy-image-pipeline
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
```
Install inside your python environment: 
```
pip intall scipy
pip install empy
pip install catkin_pkg
pip install lark-parser
```
Ros2 works with numpy 1:
```
    pip install 'numpy<2'
```

## Parameters
### Launch File Parameters
- ```env['PYTHONPATH']``` Ensures  you use your virtual environment

- ```'video_device'``` Use the camera you want 

Find video devices: 
```
ls /dev/video*
```
Verif correct video device: 
```
ffplay /dev/video0
```

## Build
Inside your ROS2 workspace folder:
```
colcon build
source install/setup.bash
```

## Launch Apriltag Navigation and Run Apriltag Server
Launch Nav2 stack and other simulation necesities:
```
ros2 launch robot_guidance_pkg nav2_launch.py 
```
Launch Apriltag Navigation:
```
ros2 launch robot_guidance_pkg apriltag_navigation_server.py 
```
Run Apriltag Client
```
ros2 run robot_guidance_pkg apriltag_navigation_client
```

## Launch Depth Controller and Run Depth Controller
Launch Depth Controller:
```
ros2 launch robot_guidance_pkg depth_controller_launch.py
```
Run Depth Controller Client:
```
ros2 run robot_guidance_pkg depth_control_client
```
Send Client Goal Request Manually: 
```
ros2 action send_goal /go_to_depth robot_guidance_interfaces/action/GoToDepth "{target_depth: 1.5}"
```

## Camera Calibration
Run Camera Calibration:
```
ros2 launch robot_guidance_pkg v4l2_camera_launch.py
```
In separate terminal:
```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 --ros-args --remap image:=/image_raw --remap camera:=/camera
```