#include <cstdlib>
#include <iostream>
#include <plan_manage/escape_search.h>

using Eigen::Vector2d;
using scan_planner::EscapeParams;
using scan_planner::EscapeResult;
using scan_planner::findEscapeTarget;
using scan_planner::StuckDetector;

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

// Obstacle points along a segment, spaced like map voxels.
static void wall(std::vector<Vector2d> &obs, const Vector2d &a, const Vector2d &b)
{
  const int n = static_cast<int>(std::ceil((b - a).norm() / 0.05));
  for (int i = 0; i <= n; ++i)
    obs.push_back(a + (b - a) * (double(i) / n));
}

int main()
{
  /* ---------- stuck detector ---------- */
  {
    StuckDetector d;
    const Vector2d p(1.0, 2.0);
    check(!d.update(0.0, p, 0.0, 10.0, 0.15, 0.5), "first update only anchors");
    check(!d.update(9.9, p + Vector2d(0.1, 0.0), 0.3, 10.0, 0.15, 0.5), "jitter below thresholds within timeout");
    check(d.update(10.1, p, 0.0, 10.0, 0.15, 0.5), "stuck after timeout");
    check(!d.update(10.2, p + Vector2d(0.2, 0.0), 0.0, 10.0, 0.15, 0.5), "moving restarts the window");
    check(!d.update(20.1, p + Vector2d(0.2, 0.0), 0.0, 10.0, 0.15, 0.5), "window restarted at 10.2");
    check(d.update(20.3, p + Vector2d(0.2, 0.0), 0.0, 10.0, 0.15, 0.5), "stuck again 10 s after restart");
    check(!d.update(20.4, p + Vector2d(0.2, 0.0), 0.6, 10.0, 0.15, 0.5), "turning restarts the window");
    check(!d.update(30.0, p + Vector2d(0.2, 0.0), 0.6 - 2.0 * M_PI, 10.0, 0.15, 0.5), "yaw compared modulo 2pi");
    d.reset();
    check(!d.update(100.0, p, 0.0, 10.0, 0.15, 0.5), "reset re-anchors");
  }

  EscapeParams p; // res 0.05, radius 1.0, offset 0.1, body 0.2, margin 0.1 -> required clearance 0.3
  const Vector2d start(0.0, 0.0);

  /* ---------- open space: nothing to escape from ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(3.0, -1.0), Vector2d(3.0, 1.0));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p);
    check(!r.found, "open space: no escape");
    check(r.start_clearance >= 0.3, "open space: clearance reported");
  }

  /* ---------- wall right in front: back up ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p);
    check(r.found, "front wall: escape found");
    check(r.start_clearance < 0.3, "front wall: start too close");
    check(r.target.x() < -0.1 && r.target.x() > -0.3 && std::abs(r.target.y()) < 0.06, "front wall: straight back ~0.15 m");
    check(r.target_clearance >= 0.3, "front wall: target clearance");
  }

  /* ---------- target_pad: the target keeps the extra clearance, trigger threshold unchanged ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    EscapeParams padded = p;
    padded.target_pad = 0.05;
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, padded);
    check(r.found && r.target_clearance >= 0.35, "pad: target clearance >= required + pad");
    check(r.target.x() < -0.15, "pad: backs up further than without pad");
  }

  /* ---------- same wall, robot facing along it (yaw = 90 deg): side step ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    const EscapeResult r = findEscapeTarget(start, M_PI / 2.0, obs, p);
    check(r.found, "wall at the side: escape found");
    check(r.target.x() < -0.0 && std::abs(r.target.y()) < 0.06, "wall at the side: lateral move away from it");
  }

  /* ---------- corner front + left: back-right ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    wall(obs, Vector2d(-2.0, 0.25), Vector2d(0.25, 0.25));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p);
    check(r.found, "corner: escape found");
    check(r.target.x() < 0.0 && r.target.y() < 0.0, "corner: moves back and to the right");
  }

  /* ---------- tight front/back, open only sideways beyond the walls' ends ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -0.4), Vector2d(0.25, 0.4));
    wall(obs, Vector2d(-0.3, -0.4), Vector2d(-0.3, 0.4));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p);
    check(r.found, "slot: escape found");
    check(std::abs(r.target.y()) > 0.6, "slot: side step out of the slot");
  }

  /* ---------- boxed in: no reachable free spot ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -0.3), Vector2d(0.25, 0.3));
    wall(obs, Vector2d(-0.3, -0.3), Vector2d(-0.3, 0.3));
    wall(obs, Vector2d(-0.3, 0.3), Vector2d(0.25, 0.3));
    wall(obs, Vector2d(-0.3, -0.3), Vector2d(0.25, -0.3));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p);
    check(!r.found, "boxed in: no escape");
  }

  /* ---------- caller's map check can veto every target ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p, [](const Vector2d &) { return false; });
    check(!r.found, "vetoed targets: no escape");
  }

  /* ---------- caller vetoes the back: picks something else, never towards the wall ---------- */
  {
    std::vector<Vector2d> obs;
    wall(obs, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    const EscapeResult r = findEscapeTarget(start, 0.0, obs, p,
                                            [](const Vector2d &t) { return t.y() > 0.3; });
    check(r.found, "partial veto: escape found");
    check(r.target.y() > 0.3 && r.target.x() < -0.1, "partial veto: back-left, not into the wall");
  }

  /* ---------- watching the remaining path: profile + sample-wise drop ---------- */
  {
    using scan_planner::firstClearanceDrop;
    using scan_planner::segmentClearanceProfile;
    const Vector2d target(-0.2, 0.0);
    std::vector<Vector2d> snap;
    const auto empty = segmentClearanceProfile(start, target, 0.0, snap, 0.1, 0.05);
    check(empty.size() == 5 && std::isinf(empty.front()), "no obstacles: infinite, 0.2 m / 0.05 m -> 5 samples");
    wall(snap, Vector2d(0.25, -2.0), Vector2d(0.25, 2.0));
    const auto planned = segmentClearanceProfile(start, target, 0.0, snap, 0.1, 0.05);
    check(std::abs(planned.front() - 0.15) < 1e-6 && std::abs(planned.back() - 0.35) < 1e-6, "wall: 0.15 at start, 0.35 at target");
    check(firstClearanceDrop(planned, segmentClearanceProfile(start, target, 0.0, snap, 0.1, 0.05), 0.35, 0.08) == -1,
          "unchanged map: no drop");

    // Someone 0.2 m behind the rear circle at the target. The path minimum is still the
    // wall (0.15) -- a min-based check would miss it; the sample-wise one does not.
    std::vector<Vector2d> live = snap;
    live.push_back(Vector2d(-0.5, 0.0));
    const auto now = segmentClearanceProfile(start, target, 0.0, live, 0.1, 0.05);
    check(*std::min_element(now.begin(), now.end()) == *std::min_element(planned.begin(), planned.end()),
          "intruder hidden behind the wall in the path minimum");
    check(firstClearanceDrop(planned, now, 0.35, 0.08) > 0, "intruder detected sample-wise");

    // A far change (beyond the cap) and voxel flicker (< eps) do not stop the robot.
    std::vector<Vector2d> far = snap;
    far.push_back(Vector2d(-1.0, 0.0));
    check(firstClearanceDrop(planned, segmentClearanceProfile(start, target, 0.0, far, 0.1, 0.05), 0.35, 0.08) == -1,
          "far obstacle ignored");
    std::vector<Vector2d> flicker = snap;
    wall(flicker, Vector2d(0.2, -2.0), Vector2d(0.2, 2.0)); // wall grows one voxel
    check(firstClearanceDrop(planned, segmentClearanceProfile(start, target, 0.0, flicker, 0.1, 0.05), 0.35, 0.08) == -1,
          "one-voxel flicker ignored");
  }

  std::cout << "escape_search_test: " << checks << " checks passed\n";
  return 0;
}
