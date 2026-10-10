#!/usr/bin/env python3
"""Headless localhost sim for the stuck escape: the robot starts 0.25 m in front of a wall
(inside the inflated map) with a far back wall so the space behind is ray-cast free.
fsm/stuck_timeout is cut to 4 s. Logs and launch file go to /tmp/navibot_escape_*.

usage (workspace root, focal schroot, after sourcing devel):
  python3 src/SCAN-Planner/src/planner/plan_manage/tests/escape_sim.py [wall_half_y] [goal_x goal_y]
e.g. "5.0 3.0 0.0" (goal behind a long wall: planner fails -> stuck -> escape backs up 0.2 m).
--intruder: as soon as the escape goal is sent, put a person-sized block right behind the
robot's escape target and slow the escape to 0.1 m/s; expect [escape] New obstacle -> ESCAPE_SAFETY."""
import math, os, signal, socket, subprocess, sys, time
import xml.etree.ElementTree as ET
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

from pathlib import Path as FilePath
PKG = str(FilePath(__file__).resolve().parents[1])
INTRUDER = '--intruder' in sys.argv
sys.argv = [a for a in sys.argv if a != '--intruder']
WALL_HALF = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
GOAL = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (3.0, 0.0)
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 11331))
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11331'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
master_log = open('/tmp/navibot_escape_master.log', 'w')
master = subprocess.Popen(['roscore', '-p', '11331'], stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
launch = None
try:
    time.sleep(2)
    rospy.init_node('escape_check', anonymous=True, disable_signals=True)
    latest = [None]
    trace = []
    cmds = []
    goals = []
    rospy.Subscriber('/quad_0/body_pose', Odometry, lambda m: latest.__setitem__(0, m))
    rospy.Subscriber('/cmd_vel', Twist, lambda m: cmds.append((time.monotonic(), m.linear.x, m.linear.y, m.angular.z)))
    rospy.Subscriber('/planning/escape_goal', PoseStamped, lambda m: goals.append((time.monotonic(), m.pose.position.x, m.pose.position.y)))
    wp_pub = rospy.Publisher('/preset_waypoints', Path, queue_size=1)
    cloud_pub = rospy.Publisher('/simulation/cloud', PointCloud2, queue_size=1)

    pts = []
    y = -WALL_HALF
    while y <= WALL_HALF:
        z = 0.0
        while z <= 1.0:
            pts.append((0.25, y, z)); pts.append((0.30, y, z)); pts.append((-3.0, y, z))
            z += 0.05
        y += 0.05

    root = ET.Element('launch')
    ET.SubElement(root, 'param', name='body_pose_topic', value='/quad_0/body_pose')
    inc = ET.SubElement(root, 'include', file=PKG + '/launch/advanced_param.xml')
    args = dict(is_real_world='false', navi_mode='2', sensor_type='lidar', body_pose_topic='/quad_0/body_pose',
                sensor_pose_topic='/quad_0/body_pose', cloud_topic='/simulation/cloud', cloud_is_world='true',
                depth_topic='/simulation/depth', cx='320', cy='240', fx='400', fy='400', max_vel='0.75')
    for k, v in args.items():
        ET.SubElement(inc, 'arg', name=k, value=v)
    ET.SubElement(root, 'param', name='scan_planner_node/fsm/stuck_timeout', value='4.0', type='double')
    if INTRUDER:
        ET.SubElement(root, 'param', name='closed_loop_controller/escape_speed', value='0.1', type='double')
    ET.SubElement(root, 'param', name='closed_loop_controller/log_dir', value='')
    ET.SubElement(root, 'node', pkg='scan_planner', type='closed_loop_controller', name='closed_loop_controller', output='screen')
    ET.SubElement(root, 'node', pkg='scan_planner', type='go2_kinematic_sim', name='go2_kinematic_sim', output='screen')
    fn = '/tmp/navibot_escape_sim.launch'
    ET.ElementTree(root).write(fn)
    log = open('/tmp/navibot_escape_sim.log', 'w')
    launch = subprocess.Popen(['roslaunch', fn], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    deadline = time.monotonic() + 15
    while (latest[0] is None or wp_pub.get_num_connections() == 0) and time.monotonic() < deadline:
        time.sleep(.1)
    assert latest[0] is not None, 'sim did not start'
    t0 = time.monotonic()
    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            h = Header(frame_id='world', stamp=rospy.Time.now())
            cloud = pts
            if INTRUDER and goals:
                # the person occludes the back wall behind it (a real lidar does not see through)
                cloud = [q for q in pts if not (q[0] < -2.0 and abs(q[1]) < 0.4)] + [(-0.5 + dx, dy, dz) for dx in (0.0, -0.05, -0.1) for dy in (-0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15)
                               for dz in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)]
            cloud_pub.publish(point_cloud2.create_cloud_xyz32(h, cloud))
            p = latest[0].pose.pose
            trace.append((time.monotonic() - t0, p.position.x, p.position.y,
                          2 * math.atan2(p.orientation.z, p.orientation.w)))
            time.sleep(.05)
    pump(3.0)
    route = Path(); route.header.frame_id = 'world'
    wp = PoseStamped(); wp.pose.position.x, wp.pose.position.y = GOAL; wp.pose.position.z = .3; wp.pose.orientation.w = 1
    route.poses.append(wp)
    wp_pub.publish(route)
    pump(25.0)
    print('escape goals:', [(round(t - t0, 1), round(x, 2), round(y, 2)) for t, x, y in goals])
    esc_cmds = [c for c in cmds if goals and c[0] >= goals[0][0] and c[0] <= goals[0][0] + 6 and (abs(c[1]) > 1e-6 or abs(c[2]) > 1e-6)]
    if esc_cmds:
        print('first escape cmd vx=%.2f vy=%.2f wz=%.2f, n=%d' % (esc_cmds[0][1:] + (len(esc_cmds),)))
    for t, x, y_, yaw in trace[::10]:
        print('t=%5.1f x=%6.2f y=%6.2f yaw=%6.2f' % (t, x, y_, yaw))
finally:
    for proc in (launch, master):
        if proc and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=15)
            except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGKILL)
