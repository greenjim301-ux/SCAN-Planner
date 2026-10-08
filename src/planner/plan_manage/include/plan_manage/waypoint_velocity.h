#ifndef SCAN_PLANNER_WAYPOINT_VELOCITY_H
#define SCAN_PLANNER_WAYPOINT_VELOCITY_H

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <vector>

namespace scan_planner
{
inline bool waypointHandoffPending(bool enabled, double terminal_speed,
                                   double elapsed, double timeout)
{
  return enabled && std::isfinite(terminal_speed) && terminal_speed > 0.05 &&
      std::isfinite(elapsed) && elapsed >= 0.0 && std::isfinite(timeout) &&
      timeout > 0.0 && timeout <= 0.15 && elapsed < timeout;
}

// This is a boundary condition for the planner, not a direct chassis command.
inline Eigen::Vector3d waypointPassVelocity(
    const Eigen::Vector3d &from, const Eigen::Vector3d &waypoint,
    const std::vector<Eigen::Vector3d> &waypoints, size_t index,
    bool enabled, double pass_speed, double max_speed, double max_acc,
    double arrival_radius, double max_slope = 0.0)
{
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  if (!enabled || index >= waypoints.size() || !from.allFinite() ||
      !waypoint.allFinite() || !std::isfinite(pass_speed) ||
      !std::isfinite(max_speed) || !std::isfinite(max_acc) ||
      !std::isfinite(arrival_radius) || pass_speed <= 0.0 ||
      max_speed <= 0.0 || max_acc <= 0.0 || arrival_radius <= 0.0)
    return zero;

  size_t next = index + 1;
  while (next < waypoints.size() && (waypoints[next] - waypoint).norm() < 0.05)
    ++next;
  if (next >= waypoints.size() || !waypoints[next].allFinite())
    return zero; // final point (including trailing duplicates)

  const Eigen::Vector3d incoming = waypoint - from;
  const Eigen::Vector3d outgoing = waypoints[next] - waypoint;
  const double in_len = incoming.norm(), out_len = outgoing.norm();
  if (in_len < 0.05 || out_len < 0.05 ||
      incoming.head<2>().norm() < 0.05 || outgoing.head<2>().norm() < 0.05)
    return zero;

  // Stairs and steep ramps keep the legacy stop: the dog should not be carried
  // onto or along them at cruise speed. The incoming slope is taken from the
  // previous waypoint (none for the first point): odom z can sit a few
  // decimetres off the route z.
  if (max_slope > 0.0)
  {
    const auto slope = [](const Eigen::Vector3d &d) {
      return std::abs(d.z()) / std::max(d.head<2>().norm(), 1e-6);
    };
    if (slope(outgoing) > max_slope)
      return zero;
    if (index > 0)
    {
      const Eigen::Vector3d incoming_route = waypoint - waypoints[index - 1];
      if (incoming_route.head<2>().norm() >= 0.05 && slope(incoming_route) > max_slope)
        return zero;
    }
  }

  const Eigen::Vector3d in_dir = incoming / in_len, out_dir = outgoing / out_len;
  const double cosine = std::clamp(in_dir.dot(out_dir), -1.0, 1.0);
  if (cosine <= -0.5 + 1e-9)
    return zero; // 120-degree turns and reversals need a stop

  double speed = std::min(pass_speed, max_speed) * (1.0 + cosine) * 0.5;
  // Leave room to stop in the next segment even if its next point is the final
  // goal. Closely spaced points must not carry a full-speed boundary condition.
  const double usable_length = std::max(0.0, std::min(in_len, out_len) - arrival_radius);
  speed = std::min(speed, std::sqrt(2.0 * max_acc * usable_length));
  if (cosine < 1.0 - 1e-6)
  {
    const double tan_half_turn = std::sqrt((1.0 - cosine) / (1.0 + cosine));
    speed = std::min(speed, std::sqrt(max_acc * arrival_radius / tan_half_turn));
  }
  if (speed < 1e-3)
    return zero;
  return (in_dir + out_dir).normalized() * speed;
}

// Keep a non-collinear boundary velocity from bending a long single quintic
// away from the route. Support points constrain the bend to the segment ends.
inline std::vector<Eigen::Vector3d> waypointReferencePoints(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &end_velocity)
{
  std::vector<Eigen::Vector3d> points;
  const Eigen::Vector3d segment = end - start;
  const double length = segment.norm();
  if (length > 0.1)
  {
    const Eigen::Vector3d direction = segment / length;
    const double blend = std::min(0.5, length * 0.25);
    const auto transverse = [&direction](const Eigen::Vector3d &v) {
      return (v - direction * v.dot(direction)).norm() > 1e-3;
    };
    if (transverse(start_velocity) || transverse(end_velocity))
    {
      // A single support point still lets the minimum-snap solution bend the
      // long interior segment. Constrain the interior as well; cap the number
      // of added points to bound reference generation cost on the board.
      const double interior_length = length - 2.0 * blend;
      const int intervals = std::min(30, std::max(1,
          static_cast<int>(std::ceil(interior_length / 1.0))));
      for (int i = 0; i <= intervals; ++i)
        points.push_back(start + direction * (blend + interior_length * i / intervals));
    }
  }
  points.push_back(end);
  return points;
}

inline Eigen::Vector3d waypointLocalTargetVelocity(
    const Eigen::Vector3d &reference_velocity, const Eigen::Vector3d &end_velocity,
    double distance_to_end, double max_speed, double max_acc)
{
  if (!reference_velocity.allFinite() || !end_velocity.allFinite() ||
      !std::isfinite(distance_to_end) || !std::isfinite(max_speed) ||
      !std::isfinite(max_acc) || max_speed <= 0.0 || max_acc <= 0.0)
    return Eigen::Vector3d::Zero();
  const double distance = std::max(0.0, distance_to_end);
  if (end_velocity.norm() < 1e-3 && distance < max_speed * max_speed / (2.0 * max_acc))
    return Eigen::Vector3d::Zero();
  // Nonzero intermediate boundary conditions survive the local horizon and
  // are still bounded by the distance available for deceleration.
  const double limit = std::min(max_speed,
      std::sqrt(end_velocity.squaredNorm() + 2.0 * max_acc * distance));
  const double speed = reference_velocity.norm();
  if (speed > limit && speed > 1e-6)
    return reference_velocity * (limit / speed);
  return reference_velocity;
}
// Duration of a straight start->end segment that leaves at v0 and arrives at v1
// (their components along the segment), accelerating towards max_speed in
// between. The rest-to-rest allocation forces a dip between two nonzero
// boundary speeds; this keeps a straight run between pass points at speed.
// Segments that end at rest keep the legacy (gentler) allocation.
inline double boundaryAwareSegmentTime(
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &end_velocity,
    double max_speed, double max_acc, double fallback)
{
  const Eigen::Vector3d segment = end - start;
  const double length = segment.norm();
  if (!segment.allFinite() || !start_velocity.allFinite() || !end_velocity.allFinite() ||
      !std::isfinite(max_speed) || !std::isfinite(max_acc) || max_speed <= 0.0 ||
      max_acc <= 0.0 || length < 1e-3)
    return fallback;
  const Eigen::Vector3d direction = segment / length;
  const double v0 = std::clamp(start_velocity.dot(direction), 0.0, max_speed);
  const double v1 = std::clamp(end_velocity.dot(direction), 0.0, max_speed);
  if (v1 < 1e-3)
    return fallback;
  double time;
  if (std::abs(v0 * v0 - v1 * v1) >= 2.0 * max_acc * length)
  {
    time = 2.0 * length / std::max(v0 + v1, 1e-3); // pure speed change
  }
  else
  {
    const double peak = std::sqrt(max_acc * length + 0.5 * (v0 * v0 + v1 * v1));
    if (peak <= max_speed)
      time = (2.0 * peak - v0 - v1) / max_acc;
    else
      time = (2.0 * max_speed - v0 - v1) / max_acc +
          (length - (2.0 * max_speed * max_speed - v0 * v0 - v1 * v1) / (2.0 * max_acc)) / max_speed;
  }
  return std::isfinite(time) && time > 1e-3 ? time : fallback;
}
} // namespace scan_planner

#endif
