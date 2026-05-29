# bringup_pkg

Project-level launch and tuning package for the current capstone stack.

This package does not replace or wrap the low-level runtime of:

- `scout_base`
- `scout_description`
- `velodyne_*`
- `pointcloud_to_laserscan`

Those should be launched and debugged independently.

This package is reserved for the upper software stack:

- `slam_toolbox`
- `nav2`

It stores only project-specific launch files and tuning files for those layers.

Current layout:

- `launch/`
  - SLAM / Nav2 entrypoints
- `config/`
  - SLAM / Nav2 tuning YAML files
- `rviz/`
  - RViz presets
- `maps/`
  - saved maps

## CCTV costmap demo

Build the CCTV demo packages from the workspace root:

```bash
colcon build --symlink-install --packages-select cctv_costmap_layer cctv_object_tools bringup_pkg
source install/setup.bash
```

Run Nav2 with the CCTV costmap layer:

```bash
ros2 launch bringup_pkg scout_cctv_nav2.launch.py
```

In another terminal, run the RViz click-to-PoseArray demo publisher:

```bash
ros2 launch cctv_object_tools rviz_click_to_pose_array.launch.py
```

Use RViz `Publish Point` with fixed frame `map`. Clicked points are published as
`geometry_msgs/msg/PoseArray` on `/cctv/objects_map`, then the custom costmap
layer marks them as obstacles before Nav2 inflation.

Clear or undo demo objects:

```bash
ros2 service call /cctv/clear_objects std_srvs/srv/Empty
ros2 service call /cctv/remove_last_object std_srvs/srv/Empty
```
