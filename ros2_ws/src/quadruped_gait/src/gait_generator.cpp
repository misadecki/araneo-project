#include "quadruped_gait/gait_generator.hh"
#include "quadruped_gait/trajectory.hh"
#include <cmath>
#include <algorithm>
#include <stdexcept>

static const Gait& validate_duty(const Gait& g) {
  if (!(g.duty > 0.0 && g.duty < 1.0))
    throw std::invalid_argument("GaitGenerator: duty out of [0.0, 1.0]");
  
  return g;
}

GaitGenerator::Legs GaitGenerator::make_legs(const Gait &g, const
    TrajectoryParams &p) {
  const FootTrajectory t = make_traj(g, p);
  return {t, t, t, t};
}

GaitGenerator::GaitGenerator(const Gait& gait, const TrajectoryParams& params,
                              double a) : g(validate_duty(gait)), p(params), 
                              a_max(a), legs(make_legs(gait, with_speed(params,
                              MIN_SPEED))) {}

double GaitGenerator::speed_eff() const {
  double dir;

  if (v != 0.0) dir = v;
  else dir = v_cmd;

  if (dir >= 0) dir = 1.0;
  else dir = -1.0;
  
  return std::copysign(std::max(std::abs(v), MIN_SPEED), dir);
}

GaitGenerator::Targets GaitGenerator::foot_targets(double dt) {
  Targets out;
  const double dv = a_max * dt;
  v += std::clamp(v_cmd - v, -dv, dv);
  
  const bool stopping = (std::abs(v_cmd) < MIN_SPEED && std::abs(v) <
                                                           MIN_SPEED);
  const bool all_idle = std::all_of(idle.begin(), idle.end(), [](bool b) {return b;});

  if (stopping && all_idle) {
    out.fill({0.0, p.z0});
    return out;
  }

  if (!stopping && all_idle) {
    phase = 0.0;
    prev_phi.fill(0.0);
    idle.fill(false);
    legs = make_legs(g, with_speed(p, speed_eff()));
  }

  phase = std::fmod(phase + dt / period(), 1.0);

  for (int i = 0; i < static_cast<int>(quadruped::LEG_COUNT); ++i) {
    const double phi = std::fmod(phase - g.offset[i] + 1.0, 1.0);
    
    if (phi < prev_phi[i]) {
      idle[i] = stopping;
      if (!stopping) legs[i] = make_traj(g, with_speed(p, speed_eff()));
    }
    
    prev_phi[i] = phi;
    if (idle[i]) out[i] = {0.0, p.z0};
    else out[i] = legs[i].at(phi * period());
  }

  return out;
}
