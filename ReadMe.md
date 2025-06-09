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
### Launch File Porametes
- ```env['PYTHONPATH'] ``` Ensures  you use your virtual environment

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

## Launch 
Launch April tag follower:
```
ros2 launch robot_guidance_pkg april_tag_follower_launch.py 
```
Launch Nav2 stack with velocity integrator:
```
ros2 launch robot_guidance_pkg nav2_launch.py 
```

# Run 
### Run Server
```
ros2 run robot_guidance_pkg depth_control_server
```

New terminal send Goal manually
```
ros2 action send_goal /go_to_depth robot_guidance_interfaces/action/GoToDepth "{target_depth: 1.5}"
```

New terminal cancel all Goals manually
```
ros2 service call /go_to_depth/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```
### Run Client
current sample goal: target_depth = 1.5
```
ros2 run robot_guidance_pkg depth_control_client
```

# To Do:
- Add Camera calibration instructions
```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera
```
- Downsize image for faster computation 