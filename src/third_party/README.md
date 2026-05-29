# third_party

Vendored ROS 2 packages used by this workspace.

## usb_cam

`usb_cam` is vendored here so this branch can build and run the camera driver
without depending on another local workspace.

Build:

```bash
colcon build --packages-select usb_cam
```

Run the copied launch file:

```bash
ros2 launch usb_cam camera.launch.py
```

The camera calibration paths use `package://usb_cam/...`, so they resolve from
the installed ROS package after a GitHub clone/build. The `video_device` values
under `usb_cam/config/params_*.yaml` are hardware device paths and may need to be
changed for a different PC or camera.
