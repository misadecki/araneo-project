#ifndef TRAJECTORY_HH
#define TRAJECTORY_HH

#include <array>

struct Quintic {
  std::array<double, 6> c = {0};

  Quintic() = default;
  Quintic(double p0, double v0, double a0, double p1, double v1, double a1,
          double T);

  double pos(double t) const;
  double vel(double t) const;
  double acc(double t) const;
};

struct TrajectoryParams {
  double q1_m, q2_st, w, Tsw, dq2;
};

class FootTrajectory {
  TrajectoryParams p;
  double Tst, T;
  Quintic q1, q2up, q2dn;

public:
  struct Joints {
    double q1, q2;
  };

  FootTrajectory(const TrajectoryParams& params);
  double period() const {return T;};
  Joints at(double t) const;
};

/* Oznaczenia 
 * c - współczynniki wielomianu
 * {p, v, a}0 - położenie, prędkość, przyspieszenie początkowe
 * {p, v, a}1 - -              ||               - końcowe
 * q1_m - maksymalna wartość kąta coxa
 * q2_m - maksymalna wartość kąta tibia
 * dq2 - zmiana kąta tibia w swingu
 * w - prędkość kątowa w stance
 * Tsw - czas swing
 * T - okres
 * h - wysokość podnoszenia stopy
 * z0 - wysokość stopy w podporze
 * Tst - czas stance
 * q1 - kąt w czasie swingu
 * q2up - wznoszenie stopy
 * q2dn - opadanie stopy
 * */

#endif // TRAJECTORY_HH
