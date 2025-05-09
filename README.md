# robot_guidance

```
pip install ultralytics
```

## Net detection
Launch net detection:
```
roslaunch robot_guidance yolo_detect.launch
```

## Waypoint Follower

```
mkdir models
```
[Download weights](https://stevens0-my.sharepoint.com/:u:/g/personal/icollado_stevens_edu/ESHtHwbcnnBIuBc8vEJ8Gj8B0TglHtRaE5QU10nU2N7RdA?e=ZItmED)\
Place weights inside models folder

Launching Waypoint Follower:
```
roslaunch robot_guidance waypoint_follower.launch
```