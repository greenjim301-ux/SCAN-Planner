# clicked_path_publisher.py

`clicked_path_publisher.py` lets you draw a path point-by-point in rviz and use
it either as the `navi_mode=3` reference path or as the `navi_mode=2` waypoint
file:

```bash
source devel/setup.bash
python3 tools/clicked_path_publisher.py                  # publish /initial_path (navi_mode=3)
python3 tools/clicked_path_publisher.py --mode keypoint  # write tools/keypoint.yaml (navi_mode=2)
```

In rviz (Fixed Frame must be `world`):

- `2D Nav Goal`: add one point per click — anywhere on the ground plane, no
  need to hit a rendered cloud point; orientation is ignored;
- `2D Pose Estimate`: finish — publish/save the collected points, then the
  buffer is cleared for the next round;
- `Publish Point`: discard the current buffer and start over.

`--mode path` needs at least 2 points, `--mode keypoint` at least 1; finishing
with fewer is rejected with a warning.

Collected points are visualized on `/clicked_path_vis` (add a `Marker` display).
Clicked z is never used — clicks easily land on obstacles or the ceiling and
would create unreachable goals in the air, so clicks only provide xy and every
point gets a fixed z per mode.

- `--mode path`: publishes `nav_msgs/Path` on `/initial_path`. Every point
  gets `--path-z` (default 0.0 m, ground level; the `navi_mode=3` planner
  adds `body_height` itself). The first point should be near the robot's
  current position (the global reference starts from it).
- `--mode keypoint`: on the closing 2D Nav Goal it does two things:
  publishes the waypoints as `nav_msgs/Path` on `/preset_waypoints`
  (`--waypoints-topic`) — a running `navi_mode=2` planner starts a new round
  immediately, and after it reaches the last waypoint it returns to
  `WAIT_TARGET`, so you can keep sending rounds; and writes
  `tools/keypoint.yaml` (`--output`, same format as `keypoint_recorder.py`)
  as a record — the planner currently does NOT load it at startup, it just
  waits on the topic. Every waypoint gets `--waypoint-z` (default 0.35 m,
  body-center height; `navi_mode=2` does NOT add `body_height`). A single
  point is allowed (one-waypoint mission). New waypoints sent mid-mission
  replace the current round; they are ignored during `EMERGENCY_STOP`
  (resend after the robot stops).

Click-based collection is for single-floor scenes — record multi-floor
waypoints by walking the robot with `keypoint_recorder.py`.

Do not run this script alongside a `navi_mode=1`/`3` planner you don't intend
to feed — they also consume `/move_base_simple/goal`. The `navi_mode=2`
planner only listens on `/preset_waypoints`, so running it while you collect
is exactly the intended workflow.

Since clicked z is ignored, clicking the ceiling is harmless (it just takes
that spot's xy). For a clearer top-down view the displayed PCD map can still
drop the ceiling: pass `pcd_z_min:=-0.5 pcd_z_max:=1.5` to `run.launch` and
`map_pub` band-passes the cloud in z before publishing
(`pcd_z_max <= pcd_z_min` disables the filter).

# publish_keypoint.py

Re-publish a stored keypoint yaml to `/preset_waypoints` so a running
`navi_mode=2` planner starts another round with the same waypoints:

```bash
source devel/setup.bash
python3 tools/publish_keypoint.py                       # tools/keypoint.yaml
python3 tools/publish_keypoint.py --file /path/to.yaml  # a specific file
```

One-shot: it waits up to `--wait` (default 5 s) for the planner to subscribe
(the topic is not latched, publishing with no subscriber would be silently
lost), publishes once, and exits. z values are published as stored. Reads the
same format `keypoint_recorder.py` and `clicked_path_publisher.py` write.

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
