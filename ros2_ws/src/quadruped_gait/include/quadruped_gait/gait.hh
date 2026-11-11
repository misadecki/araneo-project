#ifndef GAIT_HH
#define GAIT_HH

#include "quadruped_gait/trajectory.hh"
#include <array>

struct Gait {
  double duty;
  std::array<double, 4> offset; //LF, RF, LB, RB
};

inline constexpr Gait TROT = {0.5, {0.0, 0.5, 0.5, 0.0}};
inline constexpr Gait WALK = {0.75, {0.0, 0.5, 0.75, 0.25}};

inline FootTrajectory make_traj(const Gait& g, const TrajectoryParams &p) {
  const double Tst = p.Tsw * g.duty / (1.0 - g.duty);
  return FootTrajectory(p.v * Tst, p);
}

#endif // GAIT_HH
