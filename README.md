# pairs_rviz_plugins

RViz displays for visualizing PAIRS UAV data in ROS 2. This branch provides a focused set of RViz 2 plugins for rendering custom `pairs_msgs` types and textured meshes inside the PAIRS visualization stack.

## Contents

RViz displays:
- `Sphere` — renderer for `pairs_msgs/msg/Sphere` messages.
- `TexturedMeshDisplay` — display for textured meshes.

Helpers:
- `load_robot.launch` plus `generate_robot_model_xml.py` to load a UAV marker model into the TF tree.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)
```bash
sudo apt install ros-jazzy-pairs-rviz-plugins
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_rviz_plugins` package; the original
copyright is retained in [LICENSE](LICENSE).
