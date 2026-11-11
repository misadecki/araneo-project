#include "quadruped_gait/gait.hh"
#include "quadruped_gait/gait_generator.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

bool run(const std::string& name, const Gait& g, int min_expected);
double scenarios(double t);

double scenarios(double t) {
  if (t < 1.0) return 0.0; //stanie
  if (t < 6.0) return 0.1; //jazda w przód
  if (t < 10.0) return 0.0; // stop
  if (t < 16.0) return -0.1; // jazda w tył

  return 0.0; //stop
}

bool run(const std::string& name, const Gait& g, int min_expected) {
  constexpr int N = quadruped::LEG_COUNT;
  const TrajectoryParams params = {0.1, 0.3, 0.03, 0.0}; //v, Tsw, h, z0
  const double dt = 0.02, a_max = 0.2, t_end = 20.0;
  const double e = 1e-6;

  GaitGenerator gait(g, params, a_max);

  std::ofstream f("gait_" + name + ".csv");
  f << "t,v_cmd,v";
  for (int i = 0; i < N; ++i) {
    f << ",x" << i << ",z" << i;
  }

  f << '\n';

  std::array<double, N> prev_x = {0};
  std::array<double, N> max_jmp = {0};

  int min_ground = N;
  bool rest_ok = true;

  const int steps = static_cast<int>(std::lround(t_end / dt));
  for (int i = 0; i < steps; ++i) {
    const double t = i * dt;
    const double vc = scenarios(t);
    gait.set_speed(vc);
    const auto out = gait.foot_targets(dt);

    f << t << ',' << vc << ',' << gait.speed();

    for (int k = 0; k < N; ++k) {
      f << ',' << out[k].x << ',' << out[k].z;
    }

    f << '\n';

    for (int k = 0; k < N; ++k) {
      if (i > 0 ) max_jmp[k] = std::max(max_jmp[k], std::abs(out[k].x -
                                                             prev_x[k]));
      prev_x[k] = out[k].x;
    }

    if (t >= 3.0 && t < 6.0) {
      int ground = 0;
      for (int k = 0; k < N; ++k) {
        if (out[k].z <= params.z0 + e)
          ++ground;
      }

      min_ground = std::min(min_ground, ground);
    }

    if (t >= 19.0) {
      for (int k = 0; k < N; ++k)
        if (std::abs(out[k].x) > 1e-9 || std::abs(out[k].z - params.z0) > 1e-9)
          rest_ok = false;
    }
  }

  const bool ground_ok = min_ground >= min_expected;
  std::cout << "== " << name << " (T = " << gait.period() << " [s]) == \n"
    << "min legs in stance (3-6) s: " << min_ground << " (expected >= " <<
    min_expected << ") " << (ground_ok ? ":)" : ":(") << '\n' << "idle \
    position in the end: " << (rest_ok ? ":)" : ":(") << '\n' << "max dx/tick [m]:";

  for (int i = 0; i < N; ++i) {
    std::cout << ' ' << max_jmp[i];
  }
  std::cout << "\n\n";

  return ground_ok && rest_ok;
}

int main() {
  const bool ok_trot = run("trot", TROT, 2);
  const bool ok_walk = run("walk", WALK, 3);

  return (ok_trot && ok_walk) ? 0 : 1;
}
