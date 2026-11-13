#ifndef GAIT_GENERATOR_HH
#define GAIT_GENERATOR_HH

#include "quadruped_gait/gait.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include <cmath>
#include <algorithm>

class GaitGenerator {
public:
  static constexpr double MIN_SPEED = 1e-2;
  using Targets = std::array<FootTrajectory::Joints, quadruped::LEG_COUNT>;
  using Dirs = std::array<double, quadruped::LEG_COUNT>;

  GaitGenerator(const Gait& gait, const TrajectoryParams& params, double a,
                const Dirs& d);
  void set_speed(double cmd) {s_cmd = std::clamp(cmd, -1.0, 1.0);};
  double period() const {return ref.period();}
  Targets foot_targets(double dt);
  double speed() const {return s;}
private:
  Gait g;
  TrajectoryParams p;
  double phase = 0.0, s_cmd = 0.0, s = 0.0, a_max;
  Dirs dir;
  FootTrajectory ref;
  std::array<bool, quadruped::LEG_COUNT> idle = {true, true, true, true};
  std::array<double, quadruped::LEG_COUNT> prev_phi = {};
  double leg_phase(int i) const {
    return std::fmod(phase - g.offset[i] + 1.0, 1.0);
  }
};

#endif // GAIT_GENERATOR_HH
