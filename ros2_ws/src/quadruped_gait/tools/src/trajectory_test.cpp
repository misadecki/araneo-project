#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include <fstream>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include "gait_config.hh"
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 1)
    throw std::runtime_error("You need to pass YAML path.");
  const std::string path = argv[1];
  GaitConfig cfg = {};
  try {
    cfg = loadGaitConfig(path);
  } catch (const std::exception& ex) {
    std::cerr << "Error loading file: " << path << ": " << ex.what() << '\n';
    return 2;
  }

  const auto &g = quadruped::legGeometries[0];

  // q1_m, q2_st, w, Tsw, dq2
  const FootTrajectory traj(cfg.params);

  const double T = traj.period();
  const double dt = 0.001;
  const int N = static_cast<int>(2 * T / dt);

  std::ofstream f("traj.csv");
  f << "t,q1,q2,x_fk,y_fk,z_fk\n";

  for (int i = 0; i <= N; ++i) {
    const double t = i * dt;
    const auto j = traj.at(t);

    const auto pos = quadruped::legFK({j.q1, j.q2}, g);

    f << std::setprecision(12);
    f << t << ',' << j.q1 << ',' << j.q2 << ',' << pos.x << ',' << pos.y << ','
      << pos.z << '\n';
  }

  return 0;
}
