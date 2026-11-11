#ifndef GAIT_GENERATOR_HH
#define GAIT_GENERATOR_HH

#include "quadruped_gait/gait.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include <cmath>

class GaitGenerator {
  Gait g;
  TrajectoryParams p;
  double phase = 0.0, v_cmd = 0.0, v = 0.0, a_max;
  FootTrajectory ref;
  std::array<bool, quadruped::LEG_COUNT> idle = {true, true, true, true};
  std::array<double, quadruped::LEG_COUNT> prev_phi = {0};
  double leg_phase(int i) const {
    return std::fmod(phase - g.offset[i] + 1.0, 1.0);
  }
public:
  static constexpr double MIN_SPEED = 1e-3;
  using Targets = std::array<FootTrajectory::Point, quadruped::LEG_COUNT>;

  GaitGenerator(const Gait& gait, const TrajectoryParams& params, double a);
  void set_speed(double v) {v_cmd = v;};
  double period() const {return ref.period();}
  Targets foot_targets(double dt);
  double speed() const {return v;}
};

#endif // GAIT_GENERATOR_HH
