#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner/Bspline.h>
#include <scan_planner/DataDisp.h>
#include <scan_planner/PlanFinished.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/escape_search.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      ESCAPE
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoint_arrival_radius_;
    bool waypoint_continuous_;
    double waypoint_pass_speed_;
    double waypoint_max_slope_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* stuck escape: if the robot makes no progress for stuck_timeout_ while it
       has a target and sits too close to an obstacle, stop planning and
       translate it (heading kept, backing up / side stepping) to the nearest
       free spot, then resume the normal flow from GEN_NEW_TRAJ. */
    bool escape_enable_;
    double stuck_timeout_, stuck_min_dist_, stuck_min_yaw_;
    double escape_search_radius_, escape_margin_, escape_reach_dist_, escape_timeout_, escape_abort_eps_;
    bool escape_allow_unknown_;
    StuckDetector stuck_detector_;
    Eigen::Vector3d escape_target_;
    double escape_yaw_;
    std::vector<Eigen::Vector2d> escape_obstacles_; // map snapshot the escape path was checked against
    ros::Time escape_start_time_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    // In PRESET_TARGET mode, give up on the current waypoint and advance to the
    // next one after this many consecutive replan failures, instead of retrying
    // it all the way up to max_replan_fail_count_ (which drops the whole mission).
    // Must stay well below max_replan_fail_count_, which remains the final
    // safety net for when there's no next waypoint left to fall back to.
    int waypoint_skip_fail_count_{50};
    ros::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber goal_sub_, odom_sub_, path_sub_, waypoints_sub_, go2_execution_frozen_sub_, user_emergency_stop_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_, stop_pub_, finished_pub_;
    ros::Publisher escape_goal_pub_;

    /* helper functions */
    SCANPlannerManager::ReplanResult callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool planWaypointReference();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishFinished(uint8_t status); // fires once per mission end, on /planning/finished -- lets whatever
                                           // dispatched the target (e.g. deep_bridge) know it's done without polling
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    bool checkStuckAndStartEscape();
    void checkEscapeSafety();
    std::vector<Eigen::Vector2d> collectPlanarObstacles(const Eigen::Vector2d &lo, const Eigen::Vector2d &hi);
    void finishEscape(const char *reason);

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    void presetWaypointsCallback(const nav_msgs::PathConstPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg);
    void userEmergencyStopCallback(const std_msgs::EmptyConstPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
