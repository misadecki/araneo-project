#ifndef GAIT_GENERATOR_HH
#define GAIT_GENERATOR_HH

#include "quadruped_gait/gait.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"

class GaitGenerator {
  using Legs = std::array<FootTrajectory, quadruped::LEG_COUNT>;

  Gait g;
  TrajectoryParams p;
  double phase = 0.0, v_cmd = 0.0, v = 0.0, a_max;
  Legs legs;
  std::array<bool, quadruped::LEG_COUNT> idle = {true, true, true, true};
  std::array<double, quadruped::LEG_COUNT> prev_phi = {0};

  static Legs make_legs(const Gait& g, const TrajectoryParams& p);
  static TrajectoryParams with_speed(TrajectoryParams p, double v) {
    p.v = v;
    return p;
  }
  double speed_eff() const;
public:
  static constexpr double MIN_SPEED = 1e-3;
  using Targets = std::array<FootTrajectory::Point, quadruped::LEG_COUNT>;

  GaitGenerator(const Gait& gait, const TrajectoryParams& params, double a);
  void set_speed(double v) {v_cmd = v;};
  double period() const {return legs[0].period();}
  Targets foot_targets(double dt);
  double speed() const {return v;}
};

#endif // GAIT_GENERATOR_HH
