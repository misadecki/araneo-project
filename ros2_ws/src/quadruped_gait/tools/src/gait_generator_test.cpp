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

struct Segment {
  const char* name;
  double v, w;
};

static const std::array<Segment, 9> SEGMENTS = {{
  {"stand", 0.0, 0.0},
  {"forward", 1.0, 0.0},
  {"stop", 0.0, 0.0},
  {"backward", -1.0, 0.0},
  {"stop", 0.0, 0.0},
  {"turn_left", 0.0, 1.0},
  {"stop", 0.0, 0.0},
  {"arc_left", 0.5, 0.4},
  {"stop", 0.0, 0.0},
}};

static int sign_of(double x, double eps) {
  if (x > eps) return 1;
  else if (x < -eps) return -1;
  
  return 0;
}

bool run(const std::string& name, const Gait& g, int min_expected, const
         GaitConfig& cfg);

bool run(const std::string& name, const Gait& g, int min_expected, const
         GaitConfig& cfg) {
  constexpr int N = quadruped::LEG_COUNT;
  const double dt = 0.02;
  const double e = 1e-6;

  const GaitGenerator::Dirs dirs = {-1.0, 1.0, -1.0, 1.0};
  // Arbitrary, left +, right -
  const GaitGenerator::Arms arms = {0.8, -0.8, 0.8, -0.8};

  GaitGenerator gait(g, cfg.params, cfg.a_max, dirs, arms);

  std::ofstream f("gait_" + name + ".csv");
  f << "t,v_cmd,w_cmd";
  for (int i = 0; i < N; ++i) {
    f << ",s_" << i << ",q1_" << i << ",q2_" << i;
  }

  f << '\n';

  std::array<FootTrajectory::Joints, N> prev = {};
  double max_rate_q1 = 0.0, max_rate_q2 = 0.0;
  int min_ground = N;
  bool rest_ok = true, dir_ok = true;

  // 2.0 -> s [-1.0, 1.0] worst case
  const int settle = 2.0 / cfg.a_max, hold = std::max(3.0, 1.5 * gait.period());
  // one period is settle + hold
  const int steps_per_seg = static_cast<int>(std::lround((settle + hold) / dt));
  const int settle_steps = static_cast<int>(std::lround(settle / dt));
  const int total = steps_per_seg * static_cast<int>(SEGMENTS.size());

  for (int i = 0; i < total; ++i) {
    const int seg_idx = i / steps_per_seg;
    const int k_in_seg = i % steps_per_seg;
    const Segment& seg = SEGMENTS[seg_idx];
    const double t = i * dt;

    gait.set_cmd(seg.v, seg.w);
    const auto out = gait.foot_targets(dt);
    const auto& s = gait.speeds();

    f << t << ',' << seg.v << ',' << seg.w;
    for (int k = 0; k < N; ++k) {
      f << ',' << s[k] << ',' << out[k].q1 << ',' << out[k].q2;
    }
    f << '\n';

    if (i > 0) {
      for (int k = 0; k < N; ++k) {
        max_rate_q1 = std::max(max_rate_q1, std::abs(out[k].q1 - prev[k].q1) /
                               dt);
        max_rate_q2 = std::max(max_rate_q2, std::abs(out[k].q2 - prev[k].q2) /
                               dt);
      }
    }

    prev = out;
    const bool moving = (seg.v != 0.0 || seg.w != 0.0);

    if (moving && k_in_seg >= settle_steps) {
      int ground = 0;
      for (int k = 0; k < N; ++k) {
        if (std::abs(out[k].q2 - cfg.params.q2_st) <= e) ++ground;
      }
        min_ground = std::min(min_ground, ground);
    }

    if (k_in_seg == steps_per_seg - 1) {
      for (int k = 0; k < N; ++k) {
        const int expected = sign_of(seg.v - seg.w * arms[k], 1e-3);
        const int actual = sign_of(s[k], GaitGenerator::MIN_SPEED);
        if (expected != actual) dir_ok = false;
      }
    }
  }

  for (int k = 0; k < N; ++k) {
    if (std::abs(prev[k].q1) > 1e-9 || std::abs(prev[k].q2 - cfg.params.q2_st) >
    1e-9) rest_ok = false;
  }

  const double q1_max_deg = max_rate_q1 * RAD_TO_DEG;
  const double q2_max_deg = max_rate_q2 * RAD_TO_DEG;
  const bool servo_ok = (q1_max_deg <= SERVO_LIMIT_DPS) && (q2_max_deg <= SERVO_LIMIT_DPS);
  const bool ground_ok = min_ground >= min_expected;

  std::cout << "== " << name << " (T = " << gait.period() << " [s]) == \n"
    << "min legs in stance (strady state): " << min_ground << " (expected >= " <<
    min_expected << ") " << (ground_ok ? ":)" : ":(") << '\n'
    << "speed signs per segment: " << (dir_ok ? ":)" : ":(") << '\n'
    << "idle position in the end: " << (rest_ok ? ":)" : ":(") << '\n' 
    << "q1_max = " << q1_max_deg << "deg/s, q2_max = " << q2_max_deg << "deg/s" 
    << (servo_ok ? ":)" : ":(") << "\n\n";

  return ground_ok && rest_ok && servo_ok;
}

int main(int argc, char** argv) {
  if (argc < 2)
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
