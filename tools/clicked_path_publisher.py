#!/usr/bin/env python3
"""在 rviz 里逐点画路径,发布到 /initial_path(navi_mode=3)或写 keypoint.yaml(navi_mode=2)。

用法:
    python3 tools/clicked_path_publisher.py                    # mode=path, navi_mode=3 用
    python3 tools/clicked_path_publisher.py --mode keypoint    # 写 tools/keypoint.yaml, navi_mode=2 用
    python3 tools/clicked_path_publisher.py --path-topic /initial_path --frame world

rviz 操作(Fixed Frame 需为 world):
    Publish Point     逐个点出路径点,每点一次加一个;
    2D Nav Goal       作为最后一个路径点,并触发发布/写盘;完成后清空,可画下一条;
    2D Pose Estimate  放弃当前已点的点,清空重来。

已点的点在 /clicked_path_vis 上有 Marker 可视化(橙球 + 绿线),
rviz 里 Add -> Marker,话题选 /clicked_path_vis 即可实时查看。

注意:
- 点击处的 z 一律不使用:点击很容易落在障碍物/天花板上,会造出到不了的空中
  目标。两种模式的 z 都取固定值,点击只提供 xy(俯视图下点哪就是哪,误点
  天花板也只是取了该处的 xy,无害)。rviz 显示上仍建议用 map_pub 的
  pcd_z_min/pcd_z_max 把天花板切掉,便于看清地面。
- mode=path: 所有路径点 z 统一取 --path-z(默认 0.0,地面高度),navi_mode=3
  的 planner 的 pathCallback 会自动加 body_height。
- mode=keypoint: 所有 waypoint 的 z 统一取 --waypoint-z(默认 0.35,机体中心
  高度),navi_mode=2 不加 body_height。收尾时同时做两件事:
  (a) 发布到 --waypoints-topic(默认 /preset_waypoints),正在跑的 navi_mode=2
      planner 收到即开始新一轮(到达终点后回到 WAIT_TARGET,可反复发);
  (b) 写 keypoint.yaml 留档(planner 目前启动时不加载它,仅作记录)。
- 本脚本只适用于单层平面场景,多层楼请用 keypoint_recorder 遛狗录点。
- mode=path 时 planner 要求路径至少 2 个点,所以至少先 Publish Point 点 1 个,
  再用 2D Nav Goal 收尾;第一个点建议放在机器狗当前位置附近(全局参考轨迹从
  路径第一个点起算)。mode=keypoint 允许只用 2D Nav Goal 点 1 个 waypoint。
- navi_mode=1 也订阅 /move_base_simple/goal,本脚本不要和 navi_mode=1/3 的
  planner 同时跑;navi_mode=2 的 planner 只订阅 /preset_waypoints,和
  mode=keypoint 配套同时跑正是预期用法。
- mode=path 发布前确认 planner 已收到里程计,否则它会打 "No odometry yet"
  并丢弃路径。
"""

import argparse
import os

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

DEFAULT_KEYPOINT_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypoint.yaml")


def format_float(value):
    text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def atomic_write(path, content):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)

    tmp_path = path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        handle.write(content)
    os.replace(tmp_path, path)


class ClickedPathPublisher(object):
    def __init__(self, args):
        self.mode = args.mode
        self.frame = args.frame
        # 点击 z 一律不用(容易点在障碍物/天花板上),两种模式都用固定 z
        self.fixed_z = args.waypoint_z if args.mode == "keypoint" else args.path_z
        self.output_path = args.output
        self.points = []  # [(x, y, z), ...]
        self.path_pub = None
        self.wp_pub = None
        # 全部不 latch:latch 会让后启动/重启的节点立刻收到旧消息(planner 收到旧任务
        # 会自行起步,有安全隐患),代价是接收方必须先于发布时刻在线
        if self.mode == "path":
            self.path_pub = rospy.Publisher(args.path_topic, Path, queue_size=1)
        else:
            self.wp_pub = rospy.Publisher(args.waypoints_topic, Path, queue_size=1)
        self.vis_pub = rospy.Publisher("/clicked_path_vis", Marker, queue_size=4)
        rospy.Subscriber("/clicked_point", PointStamped, self.point_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.reset_cb)

    def point_cb(self, msg):
        p = (msg.point.x, msg.point.y, self.fixed_z)
        self.points.append(p)
        rospy.loginfo("加入第 %d 个路径点 [%.2f, %.2f, %.2f]", len(self.points), p[0], p[1], p[2])
        self.publish_vis()

    def goal_cb(self, msg):
        if not self.points and self.mode == "path":
            rospy.logwarn("还没有 Publish Point 点出的路径点,至少先点 1 个再用 2D Nav Goal 收尾")
            return
        end = (msg.pose.position.x, msg.pose.position.y, self.fixed_z)
        pts = self.points + [end]

        if self.mode == "path":
            self.publish_path(pts)
        else:
            self.save_keypoint_yaml(pts)

        self.points = []
        self.publish_vis()

    def make_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = rospy.Time.now()
        for x, y, z in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    def publish_path(self, pts):
        self.path_pub.publish(self.make_path(pts))
        rospy.loginfo("已发布 %d 点路径到 %s,清空待选点,可开始画下一条", len(pts), self.path_pub.name)

    def save_keypoint_yaml(self, pts):
        lines = ["fsm:", "  waypoint_num: {}".format(len(pts))]
        for index, (x, y, z) in enumerate(pts):
            lines.append("  waypoint{}_x: {}".format(index, format_float(x)))
            lines.append("  waypoint{}_y: {}".format(index, format_float(y)))
            lines.append("  waypoint{}_z: {}".format(index, format_float(z)))
        atomic_write(self.output_path, "\n".join(lines) + "\n")

        self.wp_pub.publish(self.make_path(pts))
        rospy.loginfo("已发布 %d 个 waypoint 到 %s(z 统一取 %.2f),planner 收到即开始新一轮;"
                      "同时写入 %s 留档",
                      len(pts), self.wp_pub.name, self.fixed_z, self.output_path)

    def reset_cb(self, _msg):
        self.points = []
        rospy.loginfo("已清空当前待选路径点")
        self.publish_vis()

    def publish_vis(self):
        stamp = rospy.Time.now()

        spheres = Marker()
        spheres.header.frame_id = self.frame
        spheres.header.stamp = stamp
        spheres.ns = "clicked_path"
        spheres.id = 0
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD if self.points else Marker.DELETE
        spheres.pose.orientation.w = 1.0
        spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.15
        spheres.color.r, spheres.color.g, spheres.color.b, spheres.color.a = 0.9, 0.5, 0.1, 1.0
        spheres.points = [Point(x, y, z) for x, y, z in self.points]
        self.vis_pub.publish(spheres)

        line = Marker()
        line.header.frame_id = self.frame
        line.header.stamp = stamp
        line.ns = "clicked_path"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD if len(self.points) >= 2 else Marker.DELETE
        line.pose.orientation.w = 1.0
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.8, 0.2, 1.0
        line.points = [Point(x, y, z) for x, y, z in self.points]
        self.vis_pub.publish(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["path", "keypoint"], default="keypoint",
                        help="path: 发布 nav_msgs/Path 到 --path-topic(navi_mode=3);"
                             "keypoint: 写 keypoint.yaml(navi_mode=2)")
    parser.add_argument("--path-topic", default="/initial_path", help="mode=path 时发布路径的话题")
    parser.add_argument("--output", default=DEFAULT_KEYPOINT_YAML,
                        help="mode=keypoint 时输出的 yaml 路径(默认 tools/keypoint.yaml)")
    parser.add_argument("--waypoints-topic", default="/preset_waypoints",
                        help="mode=keypoint 时发布 waypoints 的话题(navi_mode=2 planner 订阅)")
    parser.add_argument("--frame", default="world", help="路径坐标系(应与 rviz Fixed Frame 一致)")
    parser.add_argument("--waypoint-z", type=float, default=0.35,
                        help="mode=keypoint 时所有 waypoint 统一使用的 z(机体中心高度,"
                             "navi_mode=2 不加 body_height,默认 0.35 m)")
    parser.add_argument("--path-z", type=float, default=0.0,
                        help="mode=path 时所有路径点统一使用的 z(地面高度,navi_mode=3 "
                             "的 planner 会自动加 body_height,默认 0.0 m)")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("clicked_path_publisher")
    ClickedPathPublisher(args)
    if args.mode == "path":
        rospy.loginfo("clicked_path_publisher 就绪(mode=path): Publish Point 加点 | 2D Nav Goal 收尾并发布到 %s | "
                      "2D Pose Estimate 清空重来 | 可视化: /clicked_path_vis", args.path_topic)
    else:
        rospy.loginfo("clicked_path_publisher 就绪(mode=keypoint): Publish Point 加点 | 2D Nav Goal 收尾并写 %s | "
                      "2D Pose Estimate 清空重来 | 可视化: /clicked_path_vis", args.output)
    rospy.spin()


if __name__ == "__main__":
    main()
