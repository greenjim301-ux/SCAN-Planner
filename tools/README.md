# clicked_path_publisher.py

`clicked_path_publisher.py` lets you draw a reference path in rviz and publish it
as `nav_msgs/Path` on `/initial_path` for `navi_mode=3`.

```bash
source devel/setup.bash
python3 tools/clicked_path_publisher.py
```

In rviz (Fixed Frame must be `world`):

- `Publish Point`: add one waypoint per click (first point should be near the
  robot's current position);
- `2D Nav Goal`: add the final waypoint and publish the whole path, then the
  buffer is cleared for the next path;
- `2D Pose Estimate`: discard the current buffer and start over.

Collected points are visualized on `/clicked_path_vis` (add a `Marker` display).
Clicked z values are used as-is (the planner adds `body_height` itself); the
final point inherits the z of the last clicked point since 2D Nav Goal always
has z = 0. Use only with `navi_mode=3` — in `navi_mode=1` the planner consumes
`/move_base_simple/goal` too.

Clicking on ceiling points is guarded twice:

- clicks with z above `--max-z` (default 1.0 m) fall back to the previous
  point's z (`--default-z`, default 0.5 m, for the first point);
- the displayed PCD map can drop the ceiling entirely: pass
  `pcd_z_min:=-0.5 pcd_z_max:=1.5` to `run.launch` and `map_pub` band-passes
  the cloud in z before publishing, so clicks land on the floor
  (`pcd_z_max <= pcd_z_min` disables the filter).

# keypoint_recorder.py

`keypoint_recorder.py` records waypoint positions from a ROS odometry topic and writes them to `tools/keypoint.yaml`.

## 1. Start the Recorder

```bash
source devel/setup.bash
python3 tools/keypoint_recorder.py
```

By default, the recorder subscribes to:

```bash
/hand_lio/odom_vehicle
```

To use another odometry topic:

```bash
python3 tools/keypoint_recorder.py --odom /XXX
```

Note: If the robot cannot climb stairs, increase the recorded odom z height.

## 2. Keyboard Commands

After the script starts, use these keys in the terminal:

- `Enter` / `Space` / `a`: record the current position
- `r`: replace an existing waypoint with the current position
- `d`: delete an existing waypoint
- `u`: undo the last waypoint
- `l`: list recorded waypoints
- `s`: save the YAML file
- `h` / `?`: show help
- `q`: save and quit

## 3. Output Format

The default output file is `tools/keypoint.yaml`:

```yaml
fsm:
  waypoint_num: 2
  waypoint0_x: 1.0
  waypoint0_y: 2.0
  waypoint0_z: 0.5
  waypoint1_x: 3.0
  waypoint1_y: 4.0
  waypoint1_z: 0.5
```

Waypoint indices start from `waypoint0`.
