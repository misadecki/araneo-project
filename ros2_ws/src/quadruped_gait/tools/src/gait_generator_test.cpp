#include "quadruped_gait/gait.hh"
#include "quadruped_gait/gait_generator.hh"
#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include "gait_config.hh"
#include <stdexcept>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

static constexpr double RAD_TO_DEG = 57.29577951;
// Operating speed is 0.1s/60° (4.8V)
static constexpr double SERVO_LIMIT_DPS = 300.0;

bool run(const std::string& name, const Gait& g, int min_expected, const
         GaitConfig& cfg);
double scenarios(double t);

double scenarios(double t) {
  if (t < 1.0) return 0.0; //stanie
  if (t < 6.0) return 1.0; //jazda w przód
  if (t < 10.0) return 0.0; // stop
  if (t < 16.0) return -1.0; // jazda w tył

  return 0.0; //stop
}

bool run(const std::string& name, const Gait& g, int min_expected, const
         GaitConfig& cfg) {
  constexpr int N = quadruped::LEG_COUNT;
  const double dt = 0.02, t_end = 25.0;
  const double e = 1e-6;

  const GaitGenerator::Dirs dirs = {-1.0, 1.0, -1.0, 1.0};

  GaitGenerator gait(g, cfg.params, cfg.a_max, dirs);

  std::ofstream f("gait_" + name + ".csv");
  f << "t,s_cmd,s";
  for (int i = 0; i < N; ++i) {
    f << ",q1_" << i << ",q2_" << i;
  }

  f << '\n';

  std::array<FootTrajectory::Joints, N> prev = {};
  double max_rate_q1 = 0.0, max_rate_q2 = 0.0;

  int min_ground = N;
  bool rest_ok = true;

  const int steps = static_cast<int>(std::lround(t_end / dt));
  for (int i = 0; i < steps; ++i) {
    const double t = i * dt;
    const double scn = scenarios(t);
    gait.set_speed(scn);
    const auto out = gait.foot_targets(dt);

    f << t << ',' << scn << ',' << gait.speed();

    for (int k = 0; k < N; ++k) {
      f << ',' << out[k].q1 << ',' << out[k].q2;
    }

    f << '\n';

    for (int k = 0; k < N; ++k) {
      if (i > 0) {
        max_rate_q1 = std::max(max_rate_q1, std::abs(out[k].q1 - prev[k].q1) / dt);
        max_rate_q2 = std::max(max_rate_q2, std::abs(out[k].q2 - prev[k].q2) / dt);
      }
    }

    prev = out;

    if (t >= 3.0 && t < 6.0) {
      int ground = 0;
      for (int k = 0; k < N; ++k) {
        if (std::abs(out[k].q2 - cfg.params.q2_st) <= e)
          ++ground;
      }

      min_ground = std::min(min_ground, ground);
    }

    if (t >= t_end - 1.0) {
      for (int k = 0; k < N; ++k)
        if (std::abs(out[k].q1) > 1e-9 || std::abs(out[k].q2 - cfg.params.q2_st) >
          1e-9)
          rest_ok = false;
    }
  }

  const double q1_max_deg = max_rate_q1 * RAD_TO_DEG;
  const double q2_max_deg = max_rate_q2 * RAD_TO_DEG;

  const bool servo_ok = (q1_max_deg <= SERVO_LIMIT_DPS) && (q2_max_deg <= SERVO_LIMIT_DPS);
  const bool ground_ok = min_ground >= min_expected;

  std::cout << "== " << name << " (T = " << gait.period() << " [s]) == \n"
    << "min legs in stance (3-6) s: " << min_ground << " (expected >= " <<
    min_expected << ") " << (ground_ok ? ":)" : ":(") << '\n' << "idle\
    position in the end: " << (rest_ok ? ":)" : ":(") << '\n' 
    << "q1_max = " << q1_max_deg << "deg/s, q2_max = " << q2_max_deg << "deg/s" 
    << (servo_ok ? ":)" : ":(") << "\n\n";

  return ground_ok && rest_ok && servo_ok;
}

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

  const bool ok_trot = run("trot", TROT, 2, cfg);
  const bool ok_walk = run("walk", WALK, 3, cfg);

  return (ok_trot && ok_walk) ? 0 : 1;
}
