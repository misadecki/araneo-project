#ifndef GAIT_GENERATOR_HH
#define GAIT_GENERATOR_HH

#include "quadruped_gait/gait.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"

class GaitGenerator {
  Gait gait;
  TrajectoryParams p;
  double phase = 0.0;
  std::array<double, quadruped::LEG_COUNT> prev_phi = {0};
  std::array<FootTrajectory, quadruped::LEG_COUNT> legs;
public:
  static constexpr double min_speed = 1e-3;
  using Targets = std::array<FootTrajectory::Point, quadruped::LEG_COUNT>;
  GaitGenerator(const Gait& g, TrajectoryParams& p);
  void set_speed(double v) {p.v = v;};
  double period() const {return legs[0].period();};
  Targets foot_targets(double dt);
};

// Co z Trajectoryparams w trajectory?

#endif // GAIT_GENERATOR_HH
