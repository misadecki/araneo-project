#include "quadruped_gait/gait_generator.hh"
#include "quadruped_gait/trajectory.hh"
#include <algorithm>
#include <quadruped_kinematics/kinematics.hh>
#include <stdexcept>

static const Gait& validate_duty(const Gait& g, const TrajectoryParams& p) {
  if (!(g.duty > 0.0 && g.duty < 1.0))
    throw std::invalid_argument("GaitGenerator: duty out of [0.0, 1.0]");
  
  if (!(p.Tsw > 0.0 && p.q1_m > 0.0))
    throw std::invalid_argument("Tsw and q1 amplitude need to be positive.");
  return g;
}

GaitGenerator::GaitGenerator(const Gait& gait, const TrajectoryParams& params,
                              double a, const Dirs& d, const Arms& arm_) : g(validate_duty(gait, params)), p(params), 
                              a_max(a), dir(d), arm(arm_), ref(make_traj(gait, params)) {}

void GaitGenerator::set_cmd(double v, double w) {
  std::array<double, quadruped::LEG_COUNT> raw;
  double m = 1.0;
  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
    raw[i] = v - w * arm[i];
    m = std::max(m, std::abs(raw[i]));
  }

  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i)
    s_cmd[i] = raw[i] / m;
}

GaitGenerator::Targets GaitGenerator::foot_targets(double dt) {
  Targets out;
  const double ds = a_max * dt;
  bool stopping = true;

  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
    s[i] += std::clamp(s_cmd[i] - s[i], -ds, ds);
    if (std::abs(s_cmd[i]) >= MIN_SPEED || std::abs(s[i]) >= MIN_SPEED)
      stopping = false;
  }
  
  const bool all_idle = std::all_of(idle.begin(), idle.end(), [](bool b) {return b;});
  const FootTrajectory::Joints rest = {0.0, p.q2_st};

  if (stopping && all_idle) {
    out.fill(rest);
    return out;
  }

  if (all_idle) {
    phase = 0.0;
    idle.fill(false);
    for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
      prev_phi[i] = leg_phase(i);
    }
  }

  phase = std::fmod(phase + dt / ref.period(), 1.0);

  for (std::size_t i = 0; i < quadruped::LEG_COUNT; ++i) {
    const double phi = leg_phase(i);
    
    if (phi < prev_phi[i])
      idle[i] = stopping;
    
    prev_phi[i] = phi;

    if (idle[i]) out[i] = rest;
    else {
      const auto j = ref.at(phi * ref.period());
      out[i] = {dir[i] * s[i] * j.q1, j.q2};
    }
  }

  return out;
}
