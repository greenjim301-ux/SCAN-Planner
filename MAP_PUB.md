# Running map_pub standalone

`map_pub` (package `map_generator`, built from
[`src/simulator/map_generator/src/map_publisher.cpp`](src/simulator/map_generator/src/map_publisher.cpp))
loads a PCD file, optionally downsamples/offsets/z-band-passes it, and
latch-publishes it once as a `sensor_msgs/PointCloud2`. It's the node
[`simulator.xml`](src/planner/plan_manage/launch/simulator.xml) starts when
`use_pcd_map:=true`, but it has no dependency on the rest of that launch file
and can be run on its own — e.g. to just check a PCD file in rviz, or to feed
`/map_generator/global_cloud` for a real-world run without bringing up the
simulator/rendering nodes.

## Run standalone

Needs a running ROS master and the workspace sourced.

```bash
source devel/setup.bash
rosrun map_generator map_pub /path/to/your.pcd \
  _frame_id:=world \
  _cloud_topic:=/map_generator/global_cloud \
  _publish_rate:=0.1 \
  _downsample_res:=0.1 \
  _z_min:=0.0 \
  _z_max:=0.0
```

The PCD path can be given either as a plain arg (`argv[1]`, as above) or via
`_file_name:=/path/to/your.pcd` — the node checks the positional arg first,
then lets the `~file_name` param override it. It errors out if neither is
set.

Equivalent minimal launch file, mirroring what `simulator.xml` does for it:

```xml
<launch>
  <node pkg="map_generator" type="map_pub" name="map_pub" output="screen"
        args="$(find map_generator)/resource/building.pcd">
    <param name="frame_id" value="world"/>
    <param name="publish_rate" value="0.1"/>
    <param name="cloud_topic" value="/map_generator/global_cloud"/>
    <param name="downsample_res" value="0.1"/>
    <param name="z_min" value="0.0"/>
    <param name="z_max" value="0.0"/>
  </node>
</launch>
```

## Parameters

All are private (`~`) params, all optional:

| Param | Default | Meaning |
|---|---|---|
| `file_name` | *(none, or `argv[1]`)* | PCD file to load. Required (one way or the other). |
| `frame_id` | `world` | Frame stamped on the published cloud. |
| `cloud_topic` | `/map_generator/global_cloud` | Topic to publish on (latched, queue size 10). |
| `publish_rate` | `3.0` | Republish rate in Hz (latched, so a late subscriber gets the last message immediately regardless of rate; clamped to a minimum of 0.1 Hz internally). |
| `downsample_res` | `0.0` (off) | Voxel-grid leaf size; `<= 0` disables downsampling. |
| `map_offset_x/y/z` | `0.0` | Applied to every point after loading/downsampling, before the z band-pass. |
| `z_min` / `z_max` | `0.0` / `0.0` | Z band-pass applied last, in the output frame (after offset). Disabled unless `z_max > z_min`. Useful to strip the ceiling so rviz `Publish Point` clicks land on the floor, e.g. `z_min:=-0.5 z_max:=1.5`. |

NaN points are always stripped regardless of these settings. If the file
fails to load, or the cloud is empty after the z band-pass, the node logs an
error and exits(1).
