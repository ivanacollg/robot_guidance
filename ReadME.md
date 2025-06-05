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

Ros2 works with numpy 1:
```
    pip install 'numpy<2'
```

```
pip install dt-apriltags
sudo apt install ros-jazzy-usb-cam
sudo apt install ros-jazzy-apriltag-msgs
pip intall scipy
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
```
colcon build
```

## Launch 
```
ros2 launch robot_guidance april_tag_follower_launch.py 

```

# To Do:
- Add Camera calibration instructions
```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 image:=/camera/image_raw camera:=/camera
```
- Downsize image for faster computation 