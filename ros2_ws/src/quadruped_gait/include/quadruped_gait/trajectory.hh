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

class FootTrajectory {
  double L, v, Tsw, Tst, T, z0;
  Quintic cx, zup, zdn;

public:
  struct Point {
    double x, z;
  };

  FootTrajectory(double _L, double _v, double _Tsw, double _h, double _z0);
  double period() const {return T;};
  Point at(double t) const;
};

/* Oznaczenia 
 * c - współczynniki wielomianu
 * {p, v, a}0 - położenie, prędkość, przyspieszenie początkowe
 * {p, v, a}1 - -              ||               - końcowe
 * L - długość kroku
 * v - prędkość robota
 * Tsw - czas swing
 * T - okres
 * h - wysokość podnoszenia stopy
 * z0 - wysokość stopy w podporze
 * Tsw - czas stance
 * cx - położenie w czasie swingu
 * zup - wznoszenie stopy
 * zdn - opadanie stopy
 * */

#endif // TRAJECTORY_HH
