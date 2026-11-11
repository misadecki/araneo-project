#include "quadruped_gait/gait_generator.hh"
#include "quadruped_gait/trajectory.hh"
#include <algorithm>
#include <stdexcept>

static const Gait& validate_duty(const Gait& g, const TrajectoryParams& p) {
  if (!(g.duty > 0.0 && g.duty < 1.0))
    throw std::invalid_argument("GaitGenerator: duty out of [0.0, 1.0]");
  
  if (!(p.v > 0.0))
    throw std::invalid_argument("Nominal speed need to be positive.");
  return g;
}

GaitGenerator::GaitGenerator(const Gait& gait, const TrajectoryParams& params,
                              double a) : g(validate_duty(gait, params)), p(params), 
                              a_max(a), ref(make_traj(gait, params)) {}

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

  if (all_idle) {
    phase = 0.0;
    idle.fill(false);
    for (int i = 0; i < static_cast<int>(quadruped::LEG_COUNT); ++i) {
      prev_phi[i] = leg_phase(i);
    }
  }

  phase = std::fmod(phase + dt / ref.period(), 1.0);
  const double scale = v / p.v;

  for (int i = 0; i < static_cast<int>(quadruped::LEG_COUNT); ++i) {
    const double phi = leg_phase(i);
    
    if (phi < prev_phi[i])
      idle[i] = stopping;
    
    prev_phi[i] = phi;
    if (idle[i]) out[i] = {0.0, p.z0};
    else {
      const auto pt = ref.at(phi * ref.period());
      out[i] = {scale * pt.x, pt.z};
    }
  }

  return out;
}
