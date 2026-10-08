#!/usr/bin/env python3
"""Headless, localhost-only straight waypoint regression with an ideal kinematic model."""
import json, math, os, signal, subprocess, time
import xml.etree.ElementTree as ET
from pathlib import Path as FilePath
import socket
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from scan_planner.msg import PlanFinished

# Refuse to reuse an existing ROS master, including a hardware ROS network.
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 11329))

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11329'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
master_log = open('/tmp/navibot_waypoint_master.log', 'w')
master = subprocess.Popen(['roscore', '-p', '11329'], stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
launch = None
try:
    time.sleep(2)
    rospy.init_node('waypoint_continuity_check', anonymous=True, disable_signals=True)
    latest = [None]
    samples = []
    finished = []
    rospy.Subscriber('/quad_0/body_pose', Odometry, lambda m: latest.__setitem__(0, m))
    def command(m):
        if latest[0] is not None:
            p = latest[0].pose.pose.position
            samples.append((time.monotonic(), p.x, p.y, math.hypot(m.linear.x, m.linear.y)))
    rospy.Subscriber('/cmd_vel', Twist, command)
    rospy.Subscriber('/planning/finished', PlanFinished, lambda m: finished.append(m.status))
    pub = rospy.Publisher('/preset_waypoints', Path, queue_size=1)
    results = []
    for continuous in (False, True):
        root = ET.Element('launch')
        ET.SubElement(root, 'param', name='body_pose_topic', value='/quad_0/body_pose')
        inc = ET.SubElement(root, 'include', file=str(FilePath(__file__).resolve().parents[1] / 'launch' / 'advanced_param.xml'))
        args = dict(is_real_world='false', navi_mode='2', sensor_type='lidar', body_pose_topic='/quad_0/body_pose', sensor_pose_topic='/quad_0/body_pose', cloud_topic='/simulation/empty_cloud', cloud_is_world='true', depth_topic='/simulation/depth', cx='320', cy='240', fx='400', fy='400', waypoint_continuous=str(continuous).lower(), waypoint_pass_speed='0.5')
        for name, value in args.items():
            ET.SubElement(inc, 'arg', name=name, value=value)
        ET.SubElement(root, 'param', name='closed_loop_controller/log_dir', value='')
        ET.SubElement(root, 'node', pkg='scan_planner', type='closed_loop_controller', name='closed_loop_controller', output='screen')
        ET.SubElement(root, 'node', pkg='scan_planner', type='go2_kinematic_sim', name='go2_kinematic_sim', output='screen')
        filename = '/tmp/navibot_waypoint_sim.launch'
        ET.ElementTree(root).write(filename)
        log = open('/tmp/navibot_waypoint_sim_%s.log' % continuous, 'w')
        launch = subprocess.Popen(['roslaunch', filename], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        latest[0] = None
        deadline = time.monotonic() + 12
        while (latest[0] is None or pub.get_num_connections() == 0) and time.monotonic() < deadline:
            time.sleep(.1)
        assert latest[0] is not None and pub.get_num_connections(), 'simulation did not initialize'
        time.sleep(.5)
        samples.clear(); finished.clear()
        route = Path(); route.header.frame_id = 'world'; route.header.stamp = rospy.Time.now()
        for x in (1.5, 3., 4.5):
            pose = PoseStamped(); pose.pose.position.x = x; pose.pose.position.z = .3; pose.pose.orientation.w = 1
            route.poses.append(pose)
        pub.publish(route)
        deadline = time.monotonic() + 45
        while not finished and time.monotonic() < deadline:
            time.sleep(.1)
        assert finished and finished[-1] == 0, 'route did not finish successfully'
        time.sleep(4)
        windows = [[s[3] for s in samples if abs(s[1] - x) < .15] for x in (1.5, 3.)]
        assert all(windows), 'no waypoint crossing samples'
        result = dict(continuous=continuous, waypoint_min_speeds=[min(w) for w in windows], final_x=latest[0].pose.pose.position.x, final_speed=samples[-1][3], samples=len(samples))
        results.append(result)
        print(json.dumps(result), flush=True)
        if continuous:
            assert min(result['waypoint_min_speeds']) > .25, 'continuous passage slowed unexpectedly'
        assert abs(result['final_x'] - 4.5) < .2 and result['final_speed'] < .01, 'final stop incorrect'
        os.killpg(launch.pid, signal.SIGINT); launch.wait(timeout=15); launch = None; log.close()
        time.sleep(.5)
    # The ideal simulator does not necessarily reproduce the hardware pause.
    # Check nonzero intermediate passage and final stop rather than claiming
    # higher speed than the legacy planner.
    with open('/tmp/navibot_waypoint_sim_results.json', 'w') as f:
        json.dump(results, f, indent=2)
finally:
    for process in (launch, master):
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
    master_log.close()
