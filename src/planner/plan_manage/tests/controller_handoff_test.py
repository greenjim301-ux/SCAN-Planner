#!/usr/bin/env python3
"""Test real controller handoff on a private localhost ROS master, without hardware."""
import math
import os
import signal
import socket
import subprocess
import time
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from scan_planner.msg import Bspline
from std_msgs.msg import Empty

with socket.socket() as probe:
    probe.bind(('127.0.0.1', 11339))
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11339'
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
log = open('/tmp/navibot_controller_handoff.log', 'w')
master = subprocess.Popen(['roscore', '-p', '11339'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
controller = None
try:
    time.sleep(2)
    rospy.init_node('controller_handoff_test', anonymous=True, disable_signals=True)
    params = dict(time_forward=.8, heading_error_threshold=.8, kp_pos=.8, kp_yaw=1.5,
                  max_vx=.75, max_vy=.35, max_vyaw=1., finish_dist=.15,
                  finish_timeout=3., continuous_handoff=True, handoff_timeout=.15, log_dir='')
    for name, value in params.items():
        rospy.set_param('/closed_loop_controller/' + name, value)
    rospy.set_param('/body_pose_topic', '/handoff_test/odom')
    controller = subprocess.Popen(['rosrun', 'scan_planner', 'closed_loop_controller'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    odom_pub = rospy.Publisher('/handoff_test/odom', Odometry, queue_size=1)
    trajectory_pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=1)
    stop_pub = rospy.Publisher('/planning/stop', Empty, queue_size=1)
    commands = []
    rospy.Subscriber('/cmd_vel', Twist, lambda m: commands.append((time.monotonic(), math.hypot(m.linear.x, m.linear.y))))
    def odometry(_):
        m = Odometry(); m.header.stamp = rospy.Time.now(); m.header.frame_id = 'world'
        m.pose.pose.position.x = .05; m.pose.pose.position.z = .3; m.pose.pose.orientation.w = 1.
        odom_pub.publish(m)
    timer = rospy.Timer(rospy.Duration(.01), odometry)
    deadline = time.monotonic() + 10
    while (not trajectory_pub.get_num_connections() or not stop_pub.get_num_connections() or not odom_pub.get_num_connections()) and time.monotonic() < deadline:
        time.sleep(.05)
    assert trajectory_pub.get_num_connections() and odom_pub.get_num_connections(), 'controller failed to initialize'
    time.sleep(.1)
    def trajectory(identifier, moving):
        m = Bspline(); m.order = 3; m.traj_id = identifier; m.start_time = rospy.Time.now()
        m.knots = [-.3, -.2, -.1, 0., .1, .2, .3, .4, .5]
        # Cubic spline: duration 0.2 s, endpoint x=.05, terminal velocity .25 m/s.
        m.pos_pts = [Point(x=x, y=0., z=.3) for x in ([-.025, 0., .025, .05, .075] if moving else [.05] * 5)]
        return m
    start = time.monotonic(); commands.clear(); trajectory_pub.publish(trajectory(1, True))
    time.sleep(.6)
    moving = [t for t, v in commands if v > .05 and t >= start]
    assert moving, 'nonzero spline was not executed'
    zeros = [t for t, v in commands if v < .01 and t > moving[0]]
    assert zeros, 'missing bounded handoff stop'
    elapsed = zeros[0] - moving[0]
    assert .29 < elapsed < .43, 'handoff stop outside duration + grace: %.3f' % elapsed
    assert all(v < .01 for t, v in commands if t > start + .45), 'controller resumed stale trajectory'
    print('PASS: nonzero endpoint stops after %.3f s without successor' % elapsed, flush=True)
    commands.clear(); trajectory_pub.publish(trajectory(2, True)); time.sleep(.12)
    zero_start = time.monotonic(); trajectory_pub.publish(trajectory(3, False)); time.sleep(.3)
    assert any(t > zero_start + .04 for t, v in commands), 'missing commands after final spline'
    assert all(v < .01 for t, v in commands if t > zero_start + .04), 'zero-speed endpoint did not stop'
    print('PASS: zero-speed final spline bypasses handoff grace', flush=True)
    trajectory_pub.publish(trajectory(4, True)); time.sleep(.23)
    stop_start = time.monotonic(); stop_pub.publish(Empty()); time.sleep(.15)
    assert any(t > stop_start + .04 for t, v in commands), 'missing commands after explicit stop'
    assert all(v < .01 for t, v in commands if t > stop_start + .04), 'explicit stop was delayed'
    print('PASS: explicit stop interrupts handoff immediately', flush=True)
    timer.shutdown()
finally:
    for process in (controller, master):
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    log.close()
