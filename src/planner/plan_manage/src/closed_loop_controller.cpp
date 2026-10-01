#include <algorithm>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

constexpr double kMaxVYawLimit = 1.0;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Subscriber stop_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;
// Diagnostics only (no effect on control):
//   odom_cov_               localization quality, straight from /hand_lio/odom_vehicle
//   odom_msg_stamp_         stamp of the last odometry message
//   odom_pos_change_stamp_  stamp of the last message whose POSITION value differed
// The last two are different failure modes and the log must tell them apart:
// a stream that is LATE (localization backlog / stale stream) vs a stream that
// is on time but repeats the same pose (staircase content). Only the pair does.
// odom_age in the CSV is now derived from odom_pos_change_stamp_ (位置), not from
// "任一分量变过": 阶梯式输出里位置保持而偏航还在动的那些帧, 旧口径会报成"新鲜",
// 而位置环真正吃的就是这个位置值。
double odom_cov_ = std::numeric_limits<double>::quiet_NaN();
ros::Time odom_msg_stamp_;
// 位置 / 偏航**分别**记"值最后一次变化"的时刻。必须分开: 实测的阶梯式输出里
// 位置保持的那些帧偏航还在动, 若共用同一个时间戳, 位置那一跳的 dt 会被偏航刷新成
// 5 ms, 合法的分块位移就会被误判成跳变 —— 用真机数据回放验证过: 共用时间戳会拦下
// 190 次/s 并把轨迹彻底冻住; 分开之后只拦真正不可能的尖峰。
ros::Time odom_pos_change_stamp_;
ros::Time odom_yaw_change_stamp_;

// ---- 位姿不连续性诊断 (只记录, 不干预控制) ----
// 实测 (2026-10-01, lou1 上下楼, 见 nav_analysis/REPORT_20261001_lou1.md): grodom 会在
// 正常 200 Hz 位姿流上叠 ~5 次/s、0.05~0.8 m 的不连续 —— 单帧 0.41 m / 5 ms 等效
// 81 m/s, 物理不可能。控制器闭环在这条流上, 会去"追"一个并不存在的横向偏移 (实测
// 一次 +0.41 m 的横向尖峰让控制器真发出 ∫vy dt = -0.25 m 的横向指令)。
// 同一条路线两轮对照: 位置保持帧 2% (200 Hz 连续流) 时横向误差 std 0.108 m,
// 58% (阶梯流) 时 0.190/0.225 m。
//
// **这里刻意只做诊断, 不碰 p/yaw。** 曾经实现过"限幅/丢弃"的闸门并在真机数据上回放
// 验证过: 三轮数据、阈值 0.003~0.20 m 全扫, 只要还想保住位置估计的可用性 (保留
// >=95% 的交付位移), 横向误差峰值一点不降 (0.768 -> 0.768 m); 想把峰值压到 0.52 m
// 得扔 45% 的运动量, 那时位置估计已经废了。原因: 那几次摆动是十几次**同向**修正连续
// 叠加 1~2 s 形成的平台, 不是单帧尖峰 —— 速率限制只能让它慢, 慢到有意义就等于把
// 运动一起限制掉。所以控制通路上不留任何与该平台对抗的机构, 只把这个现象记录下来,
// 让离线分析 (和定位侧的改动) 有据可查。
//
// 判据是"与近期运动预测的残差": pred = 当前位置 + v_recent * (距上次位置变化的时间),
// residual = |新位置 - pred|。用残差而不是速度, 是因为这条流是阶梯式的 —— 位置刚变过
// 5 ms 又来一个 0.09 m 的大块, 用 |dp|/dt 算出来 18 m/s, 合法的分块位移会被一起标成
// 跳变; 残差口径把"保持那段"通过 v_recent 算进去了, 只有真叠上去的尖峰才留下大残差。
double odom_jump_max_residual_ = 0.20;   // [m] 残差超过它就在 CSV 的 odom_jump 列标 1
double odom_jump_window_ = 0.30;         // [s] 估 v_recent 的窗口
unsigned long odom_jump_count_ = 0;      // 本轮计数 (会话结束打一条 WARN)
bool odom_jump_this_tick_ = false;       // 本 tick 是否标了跳变, 写进 CSV
// 最近位置历史, 用来估 v_recent (只要窗口两端, 所以 deque 足够)
struct OdomHistSample
{
  ros::Time stamp;
  Eigen::Vector3d pos;
};
std::deque<OdomHistSample> odom_hist_;

double exec_time = 0.0;
ros::Time last_update_time;
// When exec_time first reached traj_duration for the current traj; zero = not
// yet. Starts the finish_timeout settle window.
ros::Time traj_end_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
// 原地转 (|yaw_err| > heading_error_threshold, 走 publishStop 那条支路) 时的偏航限幅。
// 默认等于 max_vyaw 即不改变行为; 单独给一个参数是为了让"把原地转的转速压低"只改
// 参数、不改代码。**关键**: 下发给机器人的值与 CSV 里记的值必须同源 —— 之前出现过
// 只改了 turn_cmd (那个 Twist 只喂 logRow) 而 publishStop 仍用 max_vyaw 夹过值的情况,
// 结果机器人照旧转 0.5 rad/s、日志里却写着 0.2, 日志反而不可信。
// 代价 (实测): 该值越小 -> |yaw_err| > turn_vyaw_max/kp_yaw 就顶格 -> 原地转越久 ->
// "纵向指令为 0" 的窗口越长 -> 斜面上后溜越多 (所有后溜事件都落在 frozen 窗口里)。
double turn_vyaw_max;
double finish_dist;
double finish_timeout;
std::string body_pose_topic;
// Per-session tracking log (pose + cmd_vel every control tick); empty = off.
std::string log_dir;
std::ofstream log_file;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  bool ok = true;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ok &= loadRequiredParam(nh, "time_forward", time_forward);
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam(nh, "kp_pos", kp_pos);
  ok &= loadRequiredParam(nh, "kp_yaw", kp_yaw);
  ok &= loadRequiredParam(nh, "max_vx", max_vx);
  ok &= loadRequiredParam(nh, "max_vy", max_vy);
  ok &= loadRequiredParam(nh, "max_vyaw", max_vyaw);
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  ok &= loadRequiredParam(nh, "finish_timeout", finish_timeout);
  nh.param<std::string>("log_dir", log_dir, std::string(""));
  // 位姿不连续性诊断 (只记录, 不影响控制): 两个参数都可选
  nh.param("odom_jump_max_residual", odom_jump_max_residual_, odom_jump_max_residual_);
  nh.param("odom_jump_window", odom_jump_window_, odom_jump_window_);
  if (odom_jump_max_residual_ <= 0.0 || odom_jump_window_ <= 0.0)
  {
    ROS_ERROR("[closed_loop_controller] odom_jump_max_residual=%.3f / odom_jump_window=%.3f must be > 0",
              odom_jump_max_residual_, odom_jump_window_);
    ok = false;
  }
  else
  {
    ROS_WARN("[closed_loop_controller] odom jump diagnostics ON (record only, no control effect): "
             "max_residual=%.2f m window=%.2f s",
             odom_jump_max_residual_, odom_jump_window_);
  }
  if (ok && max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  // 原地转的偏航限幅: 缺省 = max_vyaw (不改变行为)。必须放在 max_vyaw 夹过之后取默认值。
  nh.param("turn_vyaw_max", turn_vyaw_max, max_vyaw);
  if (ok && turn_vyaw_max <= 0.0)
  {
    ROS_ERROR("[closed_loop_controller] turn_vyaw_max=%.3f must be > 0", turn_vyaw_max);
    ok = false;
  }
  else if (ok && turn_vyaw_max > max_vyaw)
  {
    ROS_WARN("[closed_loop_controller] turn_vyaw_max=%.3f > max_vyaw=%.3f, has no effect",
             turn_vyaw_max, max_vyaw);
  }
  else if (ok)
  {
    ROS_WARN("[closed_loop_controller] heading-error turn yaw capped at %.2f rad/s (kp_yaw=%.2f -> "
             "saturates above %.1f deg; max_vyaw=%.2f). Longer turns = longer frozen (vx=0) windows, "
             "which is where the backward slip happens on slopes.",
             turn_vyaw_max, kp_yaw, (turn_vyaw_max / kp_yaw) * 180.0 / M_PI, max_vyaw);
  }
  return ok;
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

// ---- tracking log ----
// One CSV per tracking session: opened when a traj arrives while idle, closed
// when tracking ends (finishTracking / planning/stop). Replans during a session
// append to the same file; the traj_id column tells them apart.
void openLog()
{
  if (log_dir.empty() || log_file.is_open())
    return;

  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);
  char stamp[32];
  const std::time_t t = std::time(nullptr);
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
  // traj_id in the name: a new session can start within the same second the
  // previous one ended (next waypoint), and must not truncate its file.
  const std::string path = log_dir + "/track_" + stamp + "_traj" + std::to_string(traj_id) + ".csv";

  log_file.open(path);
  if (!log_file)
  {
    ROS_WARN("[closed_loop_controller] cannot open tracking log %s", path.c_str());
    return;
  }
  log_file << std::fixed << std::setprecision(4)
           << "t,traj_id,exec_time,traj_duration,x,y,z,yaw,x_des,y_des,z_des,yaw_des,frozen,vx,vy,vyaw"
           << ",cov,odom_stamp,odom_age,ff_x,ff_y,odom_jump\n";
  ROS_WARN("[closed_loop_controller] tracking log: %s", path.c_str());
}

void closeLog()
{
  if (log_file.is_open())
  {
    // 位姿跳变闸门拦下的次数: 这一行的位置在 CSV 里没有列, 所以同时打一条 WARN,
    // 让 journal / /rosout 也能查到 (bag 里最方便的是 CSV 的 odom_jump 列)。
    if (odom_jump_count_ > 0)
      ROS_WARN("[closed_loop_controller] odom discontinuity diagnostics: %lu steps exceeded "
               "%.2f m residual in this run (recorded only, control unaffected)",
               odom_jump_count_, odom_jump_max_residual_);
    log_file.close();
  }
}

// cmd is exactly what was published on cmd_vel this tick.
void logRow(const ros::Time &now, const Eigen::Vector3d &pos_des, double yaw_des, bool frozen,
            const geometry_msgs::Twist &cmd)
{
  if (!log_file.is_open())
    return;
  // odom_age = how old the pose POSITION VALUE is (seconds since it last changed),
  // NOT since the last message: that is the quantity the position loop acts on.
  // 用位置而不是"任一分量": 阶梯式输出里位置被保持、偏航还在动的帧, 旧口径报 5 ms
  // 会让人以为反馈是新鲜的。 -1 = 还没收到过位姿。
  // ff_* = the feed-forward part of the command (traj tangent at exec_time); the
  // P-term part is kp_pos * (pos_des - odom_pos).
  const double odom_age =
      odom_pos_change_stamp_.isZero() ? -1.0 : (now - odom_pos_change_stamp_).toSec();
  Eigen::Vector2d ff(0.0, 0.0);
  if (traj.size() > 1 && traj_duration > 0.0)
  {
    const Eigen::Vector3d vel = traj[1].evaluateDeBoorT(std::min(exec_time, traj_duration));
    ff = Eigen::Vector2d(vel(0), vel(1));
  }
  log_file << now.toSec() << ',' << traj_id << ',' << exec_time << ',' << traj_duration << ','
           << odom_pos(0) << ',' << odom_pos(1) << ',' << odom_pos(2) << ',' << odom_yaw << ','
           << pos_des(0) << ',' << pos_des(1) << ',' << pos_des(2) << ',' << yaw_des << ','
           << (frozen ? 1 : 0) << ',' << cmd.linear.x << ',' << cmd.linear.y << ',' << cmd.angular.z << ','
           << odom_cov_ << ',' << odom_msg_stamp_.toSec() << ',' << odom_age << ',' << ff(0) << ',' << ff(1) << ','
           << (odom_jump_this_tick_ ? 1 : 0)   // 本 tick 是否标了位姿不连续 (纯诊断, 位姿未做任何修改)
           << '\n';
  // Flush about once a second: the node aborts on shutdown (global ros handles
  // outliving roscpp), which would drop whatever is still buffered if the
  // planner is stopped mid-session.
  static unsigned rows = 0;
  if (++rows % 100 == 0)
    log_file.flush();
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  traj_end_time = ros::Time();
  if (!receive_traj)
    openLog();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void stopCallback(const std_msgs::EmptyConstPtr &)
{
  // receive_traj is the sole start/stop gate: clearing it here (rather than
  // adding a second flag) reuses the exact same "no traj -> publishStop()"
  // path that cmdCallback already takes at startup / before the first bspline.
  receive_traj = false;
  closeLog();
  ROS_WARN("[closed_loop_controller] received stop signal, halting trajectory tracking.");
}

// Ends tracking of the current traj for good: clears receive_traj so the
// controller stays on publishStop() -- no re-engaging when the robot drifts
// back out of finish_dist -- until the next bspline arrives.
//
// Needed because the FSM declares REACHED purely by the trajectory clock and
// never sends planning/stop on a normal finish, while exec_time here can lag
// that clock (it is frozen while turning in place). Without this, the
// controller kept chasing the endpoint indefinitely after the task ended.
void finishTracking(const Eigen::Vector2d &pos_err, bool timed_out)
{
  receive_traj = false;
  publishExecutionFrozen(false);
  publishStop();
  closeLog();
  if (timed_out)
    ROS_WARN("[closed_loop_controller] traj_id=%d: still %.3f m from the end %.1f s after the traj ended, giving up.",
             traj_id, pos_err.norm(), finish_timeout);
  else
    ROS_WARN("[closed_loop_controller] traj_id=%d finished, %.3f m from the end.", traj_id, pos_err.norm());
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  Eigen::Vector3d p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  double yaw = tf::getYaw(msg->pose.pose.orientation);
  odom_jump_this_tick_ = false;

  // ---- 位姿不连续性诊断 (只记录, 不改 p/yaw, 见文件头 odom_jump_* 的注释) ----
  // 算出"与近期运动预测的残差", 超阈值就在 CSV 的 odom_jump 列标 1 并计数。
  // 这里**没有任何控制作用**: 位姿原样使用。
  if (!odom_pos_change_stamp_.isZero())
  {
    const double dt = (msg->header.stamp - odom_pos_change_stamp_).toSec();
    if (dt > 0.0)
    {
      Eigen::Vector3d v_recent = Eigen::Vector3d::Zero();
      if (!odom_hist_.empty())
      {
        const double span = (msg->header.stamp - odom_hist_.front().stamp).toSec();
        if (span > 1e-3)
          v_recent = (odom_pos - odom_hist_.front().pos) / span;
      }
      const double residual = (p - (odom_pos + v_recent * dt)).norm();
      if (residual > odom_jump_max_residual_)
      {
        odom_jump_this_tick_ = true;
        ++odom_jump_count_;
        ROS_WARN_THROTTLE(1.0,
                          "[closed_loop_controller] odom discontinuity #%lu: residual %.3f m "
                          "(> %.2f), step %.3f m over %.0f ms, v_recent %.2f m/s "
                          "(recorded only, pose is used as-is)",
                          odom_jump_count_, residual, odom_jump_max_residual_,
                          (p - odom_pos).norm(), dt * 1e3, v_recent.norm());
      }
    }
  }

  // A repeated value is not a new pose: grodom republishes the same pose at
  // 200 Hz, so only an actual VALUE change may reset the age. 位置与偏航分开记,
  // 因为 CSV 里的 odom_age 只关心位置值有多旧 (阶梯流里位置保持而偏航还在动的帧,
  // 旧口径会报 5 ms "新鲜", 而位置环真正吃的就是这个位置值)。
  if (!have_odom || (p - odom_pos).norm() > 1e-6)
    odom_pos_change_stamp_ = msg->header.stamp;
  if (!have_odom || std::abs(normalizeAngle(yaw - odom_yaw)) > 1e-6)
    odom_yaw_change_stamp_ = msg->header.stamp;

  odom_pos = p;
  odom_yaw = yaw;
  odom_cov_ = msg->pose.covariance[0];
  odom_msg_stamp_ = msg->header.stamp;
  have_odom = true;

  // 维护 v_recent 用的位置历史 (只保留最近 odom_jump_window 秒)
  odom_hist_.push_back({msg->header.stamp, odom_pos});
  while (!odom_hist_.empty() &&
         (msg->header.stamp - odom_hist_.front().stamp).toSec() > odom_jump_window_)
    odom_hist_.pop_front();
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  // Settle window after the traj ends. Checked before the heading branch so
  // that turning in place at the end is bounded by it too.
  if (exec_time >= traj_duration)
  {
    if (traj_end_time.isZero())
      traj_end_time = now;
    else if ((now - traj_end_time).toSec() > finish_timeout)
    {
      logRow(now, pos_des, std::numeric_limits<double>::quiet_NaN(), false, geometry_msgs::Twist());
      finishTracking(Eigen::Vector2d(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1)), true);
      return;
    }
  }

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  if (std::abs(yaw_err) > heading_error_threshold)
  {
    // 原地转时的偏航限幅: 单独夹一次 (默认 turn_vyaw_max == max_vyaw, 即等价于原来),
    // 并且**下发的值与写日志的值必须是同一个** —— turn_cmd 只喂给 logRow, 一旦两者
    // 不同源, CSV 就会记下一个没发出去的值 (2026-10-01 12:10 在板子上踩过这个坑:
    // 只改了 turn_cmd=0.2, 机器人实际仍收到 0.5, 而日志写 0.2)。
    const double vyaw_turn = clamp(vyaw_cmd, -turn_vyaw_max, turn_vyaw_max);
    publishExecutionFrozen(true);
    publishStop(vyaw_turn);
    geometry_msgs::Twist turn_cmd;
    turn_cmd.angular.z = vyaw_turn; // same as publishStop(vyaw_turn)
    logRow(now, pos_des, yaw_des, true, turn_cmd);
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
  {
    logRow(now, pos_des, yaw_des, false, geometry_msgs::Twist());
    finishTracking(pos_err, false);
    return;
  }

  cmd_vel_pub.publish(cmd);
  logRow(now, pos_des, yaw_des, false, cmd);
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  stop_sub = node.subscribe("planning/stop", 10, stopCallback);
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller] ready.");

  ros::spin();
  return 0;
}
