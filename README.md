# pairs_rviz_plugins

RViz displays and interactive tools for visualizing and commanding PAIRS UAVs. The package renders custom `pairs_msgs` types in RViz and adds tools for sending goals, setting waypoints, and monitoring UAV status, plus nodelets that bridge RViz click interactions into PAIRS control topics and services.

## Contents

RViz displays:
- `Sphere`, `Bumper`, `PoseWithCovarianceArray`, `TrackArray` — renderers for the matching `pairs_msgs` types.
- `OdometryWithVelocity` — advanced `nav_msgs/Odometry` visualizer.
- `Status` — in-RViz UAV status panel mirroring the UAV status display.

RViz tools:
- `NamedSetGoal`, `ControlTool`, `WaypointPlanner` — publish goal poses, drive UAV control, and lay out flight waypoints directly in RViz.

Nodelets (`rviz_interface`):
- `RvizPoseEstimate`, `RvizNavGoal` — convert RViz "2D Pose Estimate" / "2D Nav Goal" clicks into PAIRS control references.

Helpers:
- `load_robot.launch` and `generate_robot_model_xml.py` to load a UAV marker model; `create_camera_fov_marker.launch` for camera field-of-view markers.

## Branches
- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)
```bash
sudo apt install ros-noetic-pairs-rviz-plugins
```

## Usage

Start the RViz click-to-command interface (pose-estimate and nav-goal nodelets):

```bash
roslaunch pairs_rviz_plugins rviz_interface/rviz_interface.launch
```

Load a UAV marker model into the TF tree:

```bash
roslaunch pairs_rviz_plugins load_robot.launch
```

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_rviz_plugins` package; the original
copyright is retained in [LICENSE](LICENSE).
