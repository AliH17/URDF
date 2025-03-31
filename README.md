To run, use:

```
colcon build && \
source ./install/setup.bash && \
ros2 launch my_robot robot.launch.py
```

## Changes to URDF

- Removed the mesh files
- Made a basic structure with the base link being dummy_link
- The dummy_link is connected to a box (called base_link)
- This box has 4 wheels, a camera mount on the top and two arms in the front

## Launch files
- my_realsense.launch.py launches the realsense d435i camera node.
-my_rtabmap launches rtabmap.
-robot.launch launches the robot description
