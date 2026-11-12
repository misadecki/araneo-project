#ifndef GAIT_CONFIG_HH
#define GAIT_CONFIG_HH

#include "quadruped_gait/gait.hh"
#include "quadruped_gait/trajectory.hh"
#include <string>

struct GaitConfig {
  Gait gait;
  TrajectoryParams params;
  double a_max;
};

GaitConfig loadGaitConfig(const std::string& path);

#endif // GAIT_CONFIG_HH
