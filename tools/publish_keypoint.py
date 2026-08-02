#!/usr/bin/env python3
"""把存储的 keypoint.yaml 重新发布到 /preset_waypoints,让 navi_mode=2 再跑一轮。

用法:
    python3 tools/publish_keypoint.py                       # 发 tools/keypoint.yaml
    python3 tools/publish_keypoint.py --file /path/to.yaml  # 发指定文件

一次性脚本:等 planner 订阅上(默认最多 5 s),发布一次后退出。
yaml 格式与 keypoint_recorder / clicked_path_publisher 写出的一致:

    fsm:
      waypoint_num: 2
      waypoint0_x: 1.0
      waypoint0_y: 2.0
      waypoint0_z: 0.35
      ...

z 原样发布(navi_mode=2 不加 body_height,yaml 里存的就是机体中心高度)。
"""

import argparse
import os
import sys

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DEFAULT_KEYPOINT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypoint.yaml")


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    fsm = (data or {}).get("fsm")
    if not isinstance(fsm, dict):
        raise ValueError("缺少顶层 fsm 段")

    num = int(fsm.get("waypoint_num", 0))
    if num <= 0:
        raise ValueError("fsm/waypoint_num 缺失或 <= 0")

    waypoints = []
    for i in range(num):
        try:
            waypoints.append((float(fsm["waypoint{}_x".format(i)]),
                              float(fsm["waypoint{}_y".format(i)]),
                              float(fsm["waypoint{}_z".format(i)])))
        except KeyError as exc:
            raise ValueError("缺少字段 {}".format(exc))
    return waypoints


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--file", default=DEFAULT_KEYPOINT_YAML,
                        help="keypoint yaml 路径(默认 tools/keypoint.yaml)")
    parser.add_argument("--topic", default="/preset_waypoints",
                        help="发布话题(navi_mode=2 planner 订阅)")
    parser.add_argument("--frame", default="world", help="路径坐标系")
    parser.add_argument("--wait", type=float, default=5.0,
                        help="等待订阅者连接的超时秒数(默认 5)")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        waypoints = load_waypoints(args.file)
    except (OSError, yaml.YAMLError, ValueError) as exc:
        print("读取 {} 失败: {}".format(args.file, exc), file=sys.stderr)
        sys.exit(1)

    rospy.init_node("publish_keypoint", anonymous=True)
    pub = rospy.Publisher(args.topic, Path, queue_size=1)

    # 非 latch 的一次性发布,必须等 planner 的订阅连接建立,否则消息静默丢失
    deadline = rospy.Time.now() + rospy.Duration(args.wait)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and pub.get_num_connections() == 0:
        if rospy.Time.now() > deadline:
            rospy.logerr("%.1f s 内没有订阅者连接 %s(navi_mode=2 的 planner 在跑吗?),未发布",
                         args.wait, args.topic)
            sys.exit(1)
        rate.sleep()

    path = Path()
    path.header.frame_id = args.frame
    path.header.stamp = rospy.Time.now()
    for x, y, z in waypoints:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    pub.publish(path)
    rospy.loginfo("已把 %s 的 %d 个 waypoint 发布到 %s", args.file, len(waypoints), args.topic)

    # 留一点时间让消息真正发出去再退出
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
