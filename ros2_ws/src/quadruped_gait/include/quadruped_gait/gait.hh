#ifndef GAIT_HH
#define GAIT_HH

#include "quadruped_gait/trajectory.hh"
#include <array>
#include <stdexcept>

struct Gait {
  double duty;
  std::array<double, 4> offset; //LF, RF, LB, RB
};

inline constexpr Gait TROT = {0.5, {0.0, 0.5, 0.5, 0.0}};
inline constexpr Gait WALK = {0.75, {0.0, 0.5, 0.75, 0.25}};

inline TrajectoryParams params_for(const Gait& g, TrajectoryParams p) {
  p.w = 2.0 * p.q1_m * (1.0 - g.duty) / (p.Tsw * g.duty);
  return p;
}

inline Gait gait_by_name(const std::string& name) {
  if (name == "trot") return TROT;
  else if (name == "walk") return WALK;
  else throw std::invalid_argument("Gait: walk | trot");
}

inline FootTrajectory make_traj(const Gait& g, const TrajectoryParams &p) {
  return FootTrajectory(params_for(g, p));
}

#endif // GAIT_HH
