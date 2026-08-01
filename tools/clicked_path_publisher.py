#!/usr/bin/env python3
"""在 rviz 里逐点画路径,打包发布到 /initial_path(navi_mode=3 的参考路径)。

用法:
    python3 tools/clicked_path_publisher.py
    python3 tools/clicked_path_publisher.py --path-topic /initial_path --frame world

rviz 操作(Fixed Frame 需为 world):
    Publish Point     逐个点出路径点,每点一次加一个;
    2D Nav Goal       作为最后一个路径点,并触发整条路径发布;发布后清空,可画下一条;
    2D Pose Estimate  放弃当前已点的点,清空重来。

已点的点在 /clicked_path_vis 上有 Marker 可视化(橙球 + 绿线),
rviz 里 Add -> Marker,话题选 /clicked_path_vis 即可实时查看。

注意:
- z 直接取点击处的高度(rviz 里点在点云/地面上是多少就是多少),planner 的
  pathCallback 会自动加 body_height,这里不用考虑机体高度;2D Nav Goal 本身
  z 恒为 0,所以终点的 z 取最后一个 Publish Point 的 z。
- planner 要求路径至少 2 个点,所以至少先 Publish Point 点 1 个,再用
  2D Nav Goal 收尾;第一个点建议放在机器狗当前位置附近(全局参考轨迹从
  路径第一个点起算)。
- navi_mode=1 也订阅 /move_base_simple/goal,本脚本只应在 navi_mode=3 下用,
  否则 2D Nav Goal 会被两边同时消费。
- 发布前确认 planner 已收到里程计,否则它会打 "No odometry yet" 并丢弃路径。
"""

import argparse

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class ClickedPathPublisher(object):
    def __init__(self, path_topic, frame):
        self.frame = frame
        self.points = []  # [(x, y, z), ...]
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        self.vis_pub = rospy.Publisher("/clicked_path_vis", Marker, queue_size=4, latch=True)
        rospy.Subscriber("/clicked_point", PointStamped, self.point_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.reset_cb)

    def point_cb(self, msg):
        p = (msg.point.x, msg.point.y, 0.50) # msg.point.z)
        self.points.append(p)
        rospy.loginfo("加入第 %d 个路径点 [%.2f, %.2f, %.2f]", len(self.points), p[0], p[1], p[2])
        self.publish_vis()

    def goal_cb(self, msg):
        if not self.points:
            rospy.logwarn("还没有 Publish Point 点出的路径点,至少先点 1 个再用 2D Nav Goal 收尾")
            return
        # 2D Nav Goal 的 z 恒为 0,终点 z 沿用最后一个点击点的高度
        end = (msg.pose.position.x, msg.pose.position.y, self.points[-1][2])
        pts = self.points + [end]

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
        self.path_pub.publish(path)
        rospy.loginfo("已发布 %d 点路径到 %s,清空待选点,可开始画下一条", len(pts), self.path_pub.name)

        self.points = []
        self.publish_vis()

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
    parser.add_argument("--path-topic", default="/initial_path", help="发布路径的话题")
    parser.add_argument("--frame", default="world", help="路径坐标系(应与 rviz Fixed Frame 一致)")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("clicked_path_publisher")
    ClickedPathPublisher(args.path_topic, args.frame)
    rospy.loginfo("clicked_path_publisher 就绪: Publish Point 加点 | 2D Nav Goal 收尾并发布到 %s | "
                  "2D Pose Estimate 清空重来 | 可视化: /clicked_path_vis", args.path_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
