#ifndef SCAN_PLANNER_ESCAPE_SEARCH_H
#define SCAN_PLANNER_ESCAPE_SEARCH_H

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace scan_planner
{
// Declares the robot stuck when it has neither moved nor turned noticeably for
// `timeout` seconds. The anchor is the pose at the start of the window; any
// move/turn beyond the thresholds restarts the window from the current pose.
class StuckDetector
{
public:
  void reset() { valid_ = false; }

  // Returns true once the window has lasted longer than timeout.
  bool update(double now, const Eigen::Vector2d &pos, double yaw,
              double timeout, double min_dist, double min_yaw)
  {
    const double dyaw = std::abs(std::remainder(yaw - anchor_yaw_, 2.0 * M_PI));
    if (!valid_ || (pos - anchor_pos_).norm() > min_dist || dyaw > min_yaw)
    {
      valid_ = true;
      anchor_time_ = now;
      anchor_pos_ = pos;
      anchor_yaw_ = yaw;
      return false;
    }
    return now - anchor_time_ > timeout;
  }

  double elapsed(double now) const { return valid_ ? now - anchor_time_ : 0.0; }

private:
  bool valid_{false};
  double anchor_time_{0.0};
  Eigen::Vector2d anchor_pos_{Eigen::Vector2d::Zero()};
  double anchor_yaw_{0.0};
};

struct EscapeParams
{
  double resolution{0.05};    // [m] search grid / path sampling step
  double search_radius{1.0};  // [m] max distance to the escape target
  double body_offset{0.1};    // [m] half distance between the two footprint circles (double cylinder)
  double body_radius{0.2};    // [m] footprint circle radius
  double margin{0.1};         // [m] required clearance beyond body_radius
  double clearance_eps{0.02}; // [m] how much closer than at the start the path may get to obstacles
  double target_pad{0.0};     // [m] extra clearance demanded at the target only (hysteresis against
                              //     the executor stopping short of it and re-triggering right away)
};

struct EscapeResult
{
  bool found{false};
  Eigen::Vector2d target{Eigen::Vector2d::Zero()};
  double start_clearance{0.0};
  double target_clearance{0.0};
};

// Planar distance field over a square window around a center, built from
// obstacle points. Values are capped at the window half size.
class LocalDistanceGrid
{
public:
  LocalDistanceGrid(const Eigen::Vector2d &center, double half_size, double resolution,
                    const std::vector<Eigen::Vector2d> &obstacles)
      : res_(resolution)
  {
    n_ = std::max(1, static_cast<int>(std::ceil(2.0 * half_size / res_)));
    origin_ = center - Eigen::Vector2d::Constant(0.5 * n_ * res_);
    dist_.assign(n_ * n_, half_size);

    // Only obstacles that can be closer than half_size to some cell matter.
    std::vector<Eigen::Vector2d> near;
    for (const auto &o : obstacles)
      if ((o - center).cwiseAbs().maxCoeff() < 2.0 * half_size)
        near.push_back(o);

    for (int ix = 0; ix < n_; ++ix)
      for (int iy = 0; iy < n_; ++iy)
      {
        const Eigen::Vector2d c = cellCenter(ix, iy);
        double d = dist_[ix * n_ + iy];
        for (const auto &o : near)
          d = std::min(d, (o - c).norm());
        dist_[ix * n_ + iy] = d;
      }
  }

  // Outside the window counts as touching an obstacle, so nothing beyond it is chosen.
  double at(const Eigen::Vector2d &p) const
  {
    const int ix = static_cast<int>(std::floor((p.x() - origin_.x()) / res_));
    const int iy = static_cast<int>(std::floor((p.y() - origin_.y()) / res_));
    if (ix < 0 || iy < 0 || ix >= n_ || iy >= n_)
      return 0.0;
    return dist_[ix * n_ + iy];
  }

private:
  Eigen::Vector2d cellCenter(int ix, int iy) const
  {
    return origin_ + Eigen::Vector2d((ix + 0.5) * res_, (iy + 0.5) * res_);
  }

  double res_;
  int n_;
  Eigen::Vector2d origin_;
  std::vector<double> dist_;
};

// Distance from the footprint (segment between the two circle centers, heading
// fixed at yaw) to the nearest obstacle point.
inline double footprintClearance(const LocalDistanceGrid &grid, const Eigen::Vector2d &pos,
                                 double yaw, double body_offset)
{
  const Eigen::Vector2d h(std::cos(yaw), std::sin(yaw));
  constexpr int kSamples = 5;
  double d = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kSamples; ++i)
  {
    const double s = -1.0 + 2.0 * i / (kSamples - 1);
    d = std::min(d, grid.at(pos + s * body_offset * h));
  }
  return d;
}

// Footprint clearance (heading fixed at yaw) at evenly spaced samples along the
// straight segment from -> to, both ends included, against a plain obstacle
// list (infinity where there are none). The sample count depends only on the
// geometry, so two calls with the same from/to/step line up sample by sample.
inline std::vector<double> segmentClearanceProfile(const Eigen::Vector2d &from, const Eigen::Vector2d &to, double yaw,
                                                   const std::vector<Eigen::Vector2d> &obstacles, double body_offset,
                                                   double step)
{
  const Eigen::Vector2d h(std::cos(yaw), std::sin(yaw));
  const int n = std::max(1, static_cast<int>(std::ceil((to - from).norm() / step)));
  constexpr int kSamples = 5;
  std::vector<double> profile(n + 1, std::numeric_limits<double>::infinity());
  for (int i = 0; i <= n; ++i)
  {
    const Eigen::Vector2d c = from + (to - from) * (double(i) / n);
    for (int k = 0; k < kSamples; ++k)
    {
      const Eigen::Vector2d q = c + (-1.0 + 2.0 * k / (kSamples - 1)) * body_offset * h;
      for (const auto &o : obstacles)
        profile[i] = std::min(profile[i], (o - q).norm());
    }
  }
  return profile;
}

// Footprint clearance at one pose against a plain obstacle list.
inline double footprintClearanceAt(const Eigen::Vector2d &pos, double yaw,
                                   const std::vector<Eigen::Vector2d> &obstacles, double body_offset)
{
  return segmentClearanceProfile(pos, pos, yaw, obstacles, body_offset, 1.0).front();
}

// Clearance the escape was planned with at the robot's progress along the
// original straight line start -> target (pos projected onto it, clamped to
// the segment). This is the reference a robot that drifted off the line is
// compared against: re-drawing the line from the drifted pose would compare
// the wall with itself and never notice the robot got closer to it.
inline double plannedClearanceAt(const Eigen::Vector2d &start, const Eigen::Vector2d &target,
                                 const Eigen::Vector2d &pos, double yaw,
                                 const std::vector<Eigen::Vector2d> &snapshot, double body_offset)
{
  const Eigen::Vector2d d = target - start;
  const double len2 = d.squaredNorm();
  const double t = len2 > 1e-12 ? std::max(0.0, std::min(1.0, (pos - start).dot(d) / len2)) : 0.0;
  return footprintClearanceAt(start + t * d, yaw, snapshot, body_offset);
}

// Whether the live map got closer than the snapshot anywhere along the
// remaining escape path. Compared sample by sample, not by the path minimum,
// so the obstacle right next to the start cannot mask an intrusion further
// along. Both sides are capped at `cap` (clearance beyond it does not matter);
// `eps` absorbs voxel flicker. Returns the sample index of the first drop or -1.
inline int firstClearanceDrop(const std::vector<double> &planned, const std::vector<double> &live, double cap,
                              double eps)
{
  for (size_t i = 0; i < planned.size() && i < live.size(); ++i)
    if (std::min(live[i], cap) < std::min(planned[i], cap) - eps)
      return static_cast<int>(i);
  return -1;
}

// Finds the nearest point within search_radius that the robot can reach by a
// straight-line translation while keeping its heading (backing up and side
// stepping both allowed), such that
//   - the target has clearance >= body_radius + margin + target_pad, and
//   - no point on the way is closer to obstacles than the start (minus clearance_eps),
//     i.e. the move never pushes further into the obstacle.
// `target_ok` lets the caller add map checks (inflated map free, known cell, ...).
// obstacles are planar obstacle points already filtered to the robot's height band.
inline EscapeResult findEscapeTarget(const Eigen::Vector2d &start, double yaw,
                                     const std::vector<Eigen::Vector2d> &obstacles,
                                     const EscapeParams &p,
                                     const std::function<bool(const Eigen::Vector2d &)> &target_ok = nullptr)
{
  EscapeResult result;
  const double required = p.body_radius + p.margin;
  const double half_size = p.search_radius + p.body_offset + required + p.target_pad + 2.0 * p.resolution;
  const LocalDistanceGrid grid(start, half_size, p.resolution, obstacles);

  result.start_clearance = footprintClearance(grid, start, yaw, p.body_offset);
  if (result.start_clearance >= required)
    return result; // not near any obstacle, nothing to escape from
  const double target_required = required + p.target_pad;

  const int r = static_cast<int>(std::ceil(p.search_radius / p.resolution));
  std::vector<std::pair<double, Eigen::Vector2d>> candidates;
  for (int ix = -r; ix <= r; ++ix)
    for (int iy = -r; iy <= r; ++iy)
    {
      if (ix == 0 && iy == 0)
        continue;
      const Eigen::Vector2d offset(ix * p.resolution, iy * p.resolution);
      const double d = offset.norm();
      if (d <= p.search_radius)
        candidates.emplace_back(d, start + offset);
    }
  std::sort(candidates.begin(), candidates.end(),
            [](const auto &a, const auto &b) { return a.first < b.first; });

  const double floor_clearance = result.start_clearance - p.clearance_eps;
  const double step = 0.5 * p.resolution;
  for (const auto &cand : candidates)
  {
    const Eigen::Vector2d &target = cand.second;
    const double target_clearance = footprintClearance(grid, target, yaw, p.body_offset);
    if (target_clearance < target_required)
      continue;

    bool path_ok = true;
    const int n = std::max(1, static_cast<int>(std::ceil(cand.first / step)));
    for (int i = 1; i < n && path_ok; ++i)
      path_ok = footprintClearance(grid, start + (target - start) * (double(i) / n), yaw, p.body_offset) >=
                floor_clearance;
    if (!path_ok)
      continue;

    if (target_ok && !target_ok(target))
      continue;

    result.found = true;
    result.target = target;
    result.target_clearance = target_clearance;
    return result;
  }
  return result;
}
} // namespace scan_planner

#endif
