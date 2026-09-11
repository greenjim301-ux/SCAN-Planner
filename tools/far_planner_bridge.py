#!/usr/bin/env python3
"""把 far_planner 的 /way_point 桥接到 SCAN-Planner navi_mode=1 的 /move_base_simple/goal。

分层关系:far_planner 做全局可视图规划,持续输出一个在机器人前方约
local_planner_range 处的"胡萝卜"航点;SCAN-Planner 做局部轨迹优化与避障。

需要桥接而不能直连的原因:

1. 类型   geometry_msgs/PointStamped -> geometry_msgs/PoseStamped
          (rvizGoalCallback 订阅的是 PoseStamped;姿态被忽略,只用 xy)

2. 频率   far_planner 按 main_run_freq(默认 2.5Hz)持续刷新航点,而
          waypointCallback 每收到一条就跑一次 planGlobalTraj +
          adjustGlobalTargetIfOccupied,并把 EXEC_TRAJ 打成 REPLAN_TRAJ。
          原样转发等于每 0.4s 强制重规划一次,SCAN-Planner 自己的
          fsm/thresh_replan、fsm/thresh_no_replan 全部失效。所以这里只在航点
          "移动够远"时才转发。

3. z      navi_mode=1 完全忽略消息里的 z,目标高度取首帧 body_pose 的 z
          锁存到 rviz_goal_height_(scan_replan_fsm.cpp:432)。z 唯一的作用是
          waypointCallback 开头的 `z < -0.1 -> return` 丢弃门槛。far_planner
          的航点 z = 地形高度 + vehicle_height,下楼梯到原点平面以下时会变负,
          目标就被静默丢掉了。所以这里固定发 --goal-z(默认 0.0)绕开它。

SCAN-Planner 到达目标后会 publishFinished(REACHED) 并回到 WAIT_TARGET
(scan_replan_fsm.cpp:746),不会自己继续,所以这里订阅 /planning/finished:
对方空闲时把阈值降到 --resume-dist,让下一个胡萝卜及时补上。far_planner 抵达
最终目标后胡萝卜停在原地不动,位移小于 --resume-dist 就不再补发,FSM 安静停在
WAIT_TARGET,不会出现 REACHED <-> 补发的死循环。

注意:RViz 的 2D Nav Goal 也发 /move_base_simple/goal,本脚本运行期间别用它,
否则会和 far_planner 的航点互相打架 —— 手动下目标点请用 far_planner 自己的
GoalPointTool(/goalpoint)。

用法:
    source devel/setup.bash
    python3 tools/far_planner_bridge.py
    python3 tools/far_planner_bridge.py --min-update-dist 1.5
"""

import argparse
import math

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

try:
    from scan_planner.msg import PlanFinished
except ImportError:  # 没 source devel/setup.bash 时退化为纯距离触发
    PlanFinished = None


class FarPlannerBridge(object):
    def __init__(self, args):
        self.args = args
        self.latest = None          # 最近一次收到的 far_planner 航点 (x, y, z)
        self.last_sent = None       # 最近一次转发出去的航点
        self.last_send_time = rospy.Time(0)
        self.planner_idle = True    # SCAN-Planner 是否在 WAIT_TARGET
        self.sent_count = 0

        self.pub = rospy.Publisher(args.output_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(args.waypoint_topic, PointStamped, self.waypoint_cb, queue_size=1)

        if PlanFinished is not None:
            rospy.Subscriber("/planning/finished", PlanFinished, self.finished_cb, queue_size=10)
        else:
            rospy.logwarn("导入 scan_planner.msg.PlanFinished 失败,不订阅 /planning/finished;"
                          "到达目标后要靠 --min-update-dist 触发下一次转发,衔接会略迟钝")

        self.timer = rospy.Timer(rospy.Duration(1.0 / args.rate), self.tick)

    def waypoint_cb(self, msg):
        self.latest = (msg.point.x, msg.point.y, msg.point.z)

    def finished_cb(self, msg):
        # REACHED 和 EMERGENCY_STOP 都会让 FSM 回到 WAIT_TARGET
        self.planner_idle = True
        rospy.loginfo("SCAN-Planner 回到 WAIT_TARGET (status=%d),等待下一个目标", msg.status)

    def tick(self, _event):
        if self.latest is None:
            return

        now = rospy.Time.now()
        if (now - self.last_send_time).to_sec() < self.args.min_interval:
            return

        # 开机前几次无条件重发:SCAN-Planner 在收到首帧 body_pose 之前
        # (rviz_height_ready_ 为 false)会直接丢弃目标,只发一次可能正好被丢掉
        if self.sent_count < self.args.startup_repeat:
            self.publish(now)
            return

        # 空闲时用小阈值尽快续上,忙碌时用大阈值避免打断对方的重规划
        thresh = self.args.resume_dist if self.planner_idle else self.args.min_update_dist

        if self.last_sent is not None:
            dx = self.latest[0] - self.last_sent[0]
            dy = self.latest[1] - self.last_sent[1]
            if math.hypot(dx, dy) < thresh:
                return

        self.publish(now)

    def publish(self, stamp):
        x, y, _z = self.latest

        goal = PoseStamped()
        goal.header.frame_id = self.args.frame
        goal.header.stamp = stamp
        goal.pose.position.x = x
        goal.pose.position.y = y
        # navi_mode=1 忽略这个 z(高度用 rviz_goal_height_),这里只要避开
        # waypointCallback 的 `z < -0.1 -> return` 丢弃门槛
        goal.pose.position.z = self.args.goal_z
        goal.pose.orientation.w = 1.0

        self.pub.publish(goal)
        self.last_sent = self.latest
        self.last_send_time = stamp
        self.planner_idle = False
        self.sent_count += 1
        rospy.loginfo("转发目标 -> %s: [%.2f, %.2f]", self.args.output_topic, x, y)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--waypoint-topic", default="/way_point",
                        help="far_planner 的航点话题 (geometry_msgs/PointStamped)")
    parser.add_argument("--output-topic", default="/move_base_simple/goal",
                        help="SCAN-Planner navi_mode=1 订阅的话题 (geometry_msgs/PoseStamped)")
    parser.add_argument("--frame", default="world",
                        help="输出的 frame_id。navi_mode=1 只取 xy、不看 frame,"
                             "这里只影响 rviz 显示")
    parser.add_argument("--goal-z", type=float, default=0.0,
                        help="填进消息的 z。navi_mode=1 忽略它(高度取首帧 body_pose 的 z),"
                             "只需大于 -0.1 以免被 waypointCallback 丢弃")
    parser.add_argument("--min-update-dist", type=float, default=1.0,
                        help="SCAN-Planner 忙碌时,航点至少移动这么远(xy 平面)才转发。"
                             "默认对齐 fsm/thresh_replan=1.0")
    parser.add_argument("--resume-dist", type=float, default=0.3,
                        help="SCAN-Planner 处于 WAIT_TARGET 时改用的小阈值。设得过小会在"
                             "最终目标处反复补发")
    parser.add_argument("--min-interval", type=float, default=0.5,
                        help="两次转发之间的最小间隔秒数,兜底限频")
    parser.add_argument("--startup-repeat", type=int, default=3,
                        help="开机时无条件重发的次数,避开 rviz_height_ready_ 未就绪导致的丢弃")
    parser.add_argument("--rate", type=float, default=10.0, help="内部检查频率 Hz")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("far_planner_bridge")
    FarPlannerBridge(args)
    rospy.loginfo("far_planner 桥接已启动: %s -> %s (navi_mode=1)",
                  args.waypoint_topic, args.output_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
