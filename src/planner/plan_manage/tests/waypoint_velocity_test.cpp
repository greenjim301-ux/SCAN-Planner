#include <iostream>
#include <cstdlib>
#include <limits>
#include <plan_manage/waypoint_velocity.h>
#include <traj_utils/polynomial_traj.h>

using Eigen::Vector3d;
using scan_planner::waypointPassVelocity;
using scan_planner::waypointLocalTargetVelocity;
using scan_planner::waypointReferencePoints;

static int checks = 0;
static void check(bool value, const char *message)
{
  ++checks;
  if (!value)
  {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

// Exercise the actual polynomial generator used by the FSM, including its
// reference support points and first/last segment time allocation.
static PolynomialTraj reference(const Vector3d &start, const Vector3d &end,
                                const Vector3d &start_vel, const Vector3d &end_vel)
{
  const auto support = waypointReferencePoints(start, end, start_vel, end_vel);
  std::vector<Vector3d> points{start};
  for (const auto &p : support)
  {
    const Vector3d prev = points.back();
    const double length = (p - prev).norm();
    const int segments = length > 4.0 ? static_cast<int>(std::floor(length / 4.0)) + 1 : 1;
    for (int i = 1; i <= segments; ++i)
      points.push_back(prev + (p - prev) * (static_cast<double>(i) / segments));
  }
  Eigen::MatrixXd positions(3, points.size());
  Eigen::VectorXd times(points.size() - 1);
  for (size_t i = 0; i < points.size(); ++i)
  {
    positions.col(i) = points[i];
    if (i > 0)
      times(i - 1) = (points[i] - points[i - 1]).norm() / 0.75;
  }
  times(0) *= 2.0;
  if (times.size() > 1)
    times(times.size() - 1) *= 2.0;
  auto trajectory = positions.cols() > 2
      ? PolynomialTraj::minSnapTraj(positions, start_vel, end_vel, Vector3d::Zero(), Vector3d::Zero(), times)
      : PolynomialTraj::one_segment_traj_gen(start, start_vel, Vector3d::Zero(), end, end_vel, Vector3d::Zero(), times(0));
  trajectory.init();
  return trajectory;
}

int main()
{
  const Vector3d origin = Vector3d::Zero(), point(4, 0, 0);
  const std::vector<Vector3d> straight{point, Vector3d(8, 0, 0)};
  auto pass = [&](const std::vector<Vector3d> &route, size_t index = 0, bool enabled = true) {
    return waypointPassVelocity(origin, route[index], route, index, enabled, 0.5, 0.75, 2.0, 0.3);
  };
  const Vector3d straight_vel = pass(straight);
  check((straight_vel - Vector3d(0.5, 0, 0)).norm() < 1e-9, "straight point carries 0.5 m/s");
  check(pass(straight, 1).isZero(), "final point stops");
  check(pass(straight, 0, false).isZero(), "legacy switch stops intermediate point");
  check((pass({point, point, straight.back()}) - straight_vel).norm() < 1e-9, "duplicate next point is skipped");
  check(pass({point, point}).isZero(), "trailing duplicates still stop");
  const Vector3d right_vel = pass({point, Vector3d(4, 4, 0)});
  check(std::abs(right_vel.norm() - 0.25) < 1e-9 && right_vel.x() > 0 && right_vel.y() > 0,
        "90-degree turn slows and uses bisector tangent");
  check(pass({point, Vector3d(0, 0, 0)}).isZero(), "reversal stops");
  check(pass({point, point + Vector3d(-2, std::sqrt(12.0), 0)}).isZero(), "120-degree turn stops");
  check(pass({point, Vector3d(4.31, 0, 0)}).norm() <= 0.201, "short outgoing segment limits stop distance");
  check(pass({point, Vector3d(4.1, 0, 0)}).isZero(), "overlapping arrival regions stop");
  const Vector3d slope_vel = pass({Vector3d(4, 0, 2), Vector3d(8, 0, 4)});
  check(std::abs(slope_vel.norm() - 0.5) < 1e-9 && slope_vel.z() > 0, "slope preserves 3D tangent");
  check(pass({Vector3d(0, 0, 2), Vector3d(0, 0, 4)}).isZero(), "vertical-only segment stops");
  check(waypointPassVelocity(origin, point, straight, 0, true,
        std::numeric_limits<double>::quiet_NaN(), 0.75, 2.0, 0.3).isZero(), "invalid limit rejected");
  check(waypointPassVelocity(origin, point, straight, 0, true, 0.5, 0.2, 2.0, 0.3).norm() <= 0.2,
        "planner speed limit respected");
  check((waypointLocalTargetVelocity(straight_vel, straight_vel, 0.0, 0.75, 2.0) - straight_vel).norm() < 1e-9,
        "local horizon retains intermediate terminal speed");
  check(waypointLocalTargetVelocity(straight_vel, origin, 0.0, 0.75, 2.0).isZero(), "final / obstacle target brakes");
  check(waypointLocalTargetVelocity(Vector3d(0.75, 0, 0), right_vel, 0.01, 0.75, 2.0).norm() <= std::sqrt(0.25 * 0.25 + 0.04) + 1e-9,
        "local speed can decelerate to corner boundary");
  auto sloped = [&](const std::vector<Vector3d> &route, size_t index) {
    return waypointPassVelocity(origin, route[index], route, index, true, 0.9, 0.9, 2.0, 0.3, 0.15);
  };
  const std::vector<Vector3d> stairs{Vector3d(1.5, 0, 0), Vector3d(3, 0, 0.1), Vector3d(4.5, 0, 0.7),
                                     Vector3d(6, 0, 1.3), Vector3d(7.5, 0, 1.3), Vector3d(9, 0, 1.3)};
  check(sloped(stairs, 0).norm() > 0.85, "flat point before a gentle rise keeps speed");
  check(sloped(stairs, 1).isZero(), "point before stairs stops");
  check(sloped(stairs, 2).isZero(), "point on stairs stops");
  check(sloped(stairs, 3).isZero(), "point leaving stairs stops");
  check(sloped(stairs, 4).norm() > 0.85, "flat point after stairs keeps speed");
  check(waypointPassVelocity(Vector3d(0, 0, 0.4), stairs[0], stairs, 0, true, 0.9, 0.9, 2.0, 0.3, 0.15).norm() > 0.5,
        "odom z offset on the first incoming segment is not treated as stairs");

  using scan_planner::boundaryAwareSegmentTime;
  const Vector3d x_end(1.8, 0, 0), cruise(0.9, 0, 0);
  check(std::abs(boundaryAwareSegmentTime(origin, x_end, cruise, cruise, 0.9, 2.0, -1.0) - 2.0) < 1e-9,
        "cruise-to-cruise segment takes length / max speed");
  check(boundaryAwareSegmentTime(origin, x_end, cruise, origin, 0.9, 2.0, -1.0) == -1.0,
        "segment ending at rest keeps legacy allocation");
  const double accelerate = boundaryAwareSegmentTime(origin, x_end, Vector3d(0.3, 0, 0), cruise, 0.9, 2.0, -1.0);
  check(std::abs(accelerate - (0.3 + (1.8 - (0.81 - 0.09) / 4.0) / 0.9)) < 1e-9, "trapezoid from a lower start speed");
  const double short_time = boundaryAwareSegmentTime(origin, Vector3d(0.2, 0, 0), Vector3d(0.5, 0, 0), Vector3d(0.5, 0, 0), 0.9, 2.0, -1.0);
  check(std::abs(short_time - 2.0 * (std::sqrt(0.4 + 0.25) - 0.5) / 2.0) < 1e-9, "triangle when max speed is unreachable");
  check(std::abs(boundaryAwareSegmentTime(origin, Vector3d(0.1, 0, 0), cruise, Vector3d(0.1, 0, 0), 0.9, 2.0, -1.0) - 0.2) < 1e-9,
        "pure deceleration uses mean speed");
  check(std::abs(boundaryAwareSegmentTime(origin, x_end, -cruise, cruise, 0.9, 2.0, -1.0) -
                 boundaryAwareSegmentTime(origin, x_end, origin, cruise, 0.9, 2.0, -1.0)) < 1e-9,
        "backward start velocity counts as rest");
  check(boundaryAwareSegmentTime(origin, origin, cruise, cruise, 0.9, 2.0, -1.0) == -1.0, "degenerate segment falls back");
  auto steady = PolynomialTraj::one_segment_traj_gen(origin, cruise, origin, x_end, cruise, origin,
      boundaryAwareSegmentTime(origin, x_end, cruise, cruise, 0.9, 2.0, -1.0));
  steady.init();
  double min_speed = 1e9;
  for (int i = 0; i <= 100; ++i)
    min_speed = std::min(min_speed, steady.evaluateVel(steady.getTimeSum() * i / 100.0).norm());
  check(min_speed > 0.9 - 1e-6, "straight initial polynomial between pass points has no speed dip");

  check(scan_planner::waypointHandoffPending(true, 0.25, 0.01, 0.15), "nonzero endpoint can await next spline");
  check(!scan_planner::waypointHandoffPending(true, 0.25, 0.15, 0.15), "handoff stops at deadline");
  check(!scan_planner::waypointHandoffPending(true, 0.0, 0.01, 0.15), "final zero endpoint stops immediately");
  check(!scan_planner::waypointHandoffPending(false, 0.25, 0.01, 0.15), "legacy controller handoff disabled");
  check(!scan_planner::waypointHandoffPending(true, 0.25, 0.01, 0.3), "unbounded handoff configuration rejected");

  auto incoming = reference(origin, point, origin, straight_vel);
  auto outgoing = reference(point, straight.back(), straight_vel, origin);
  check((incoming.evaluateVel(incoming.getTimeSum()) - outgoing.evaluateVel(0.0)).norm() < 1e-7,
        "straight segment join has continuous nonzero velocity");
  check(outgoing.evaluateVel(outgoing.getTimeSum()).norm() < 1e-7, "actual final polynomial stops");
  double corner_deviation = 0.0;
  for (double length : {0.31, 0.6, 1.0, 4.0, 10.0, 40.0})
  {
    const Vector3d end(length, 0, 0);
    auto corner = reference(origin, end, origin, right_vel);
    auto after_corner = reference(end, end + Vector3d(0, length, 0), right_vel, origin);
    check((corner.evaluateVel(corner.getTimeSum()) - after_corner.evaluateVel(0.0)).norm() < 1e-6,
          "corner reference segments retain matching derivatives");
    for (int i = 0; i <= 1000; ++i)
    {
      const auto p = corner.evaluate(corner.getTimeSum() * i / 1000.0);
      const auto q = after_corner.evaluate(after_corner.getTimeSum() * i / 1000.0);
      corner_deviation = std::max(corner_deviation, std::max(std::abs(p.y()), std::abs(q.x() - length)));
      check(p.allFinite() && q.allFinite(), "corner polynomials finite");
    }
  }
  std::cout << "corner deviation " << corner_deviation << " m\n";
  check(corner_deviation < 0.3, "support points bound corner reference deviation in sampled scenarios");
  std::cout << "PASS: " << checks << " checks; corner reference max deviation " << corner_deviation << " m\n";
}
