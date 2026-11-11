#include "quadruped_gait/trajectory.hh"
#include <cmath>
#include <stdexcept>

Quintic::Quintic(double p0, double v0, double a0, double p1, double v1, double a1,
        double T) {
  const double d = p1 - p0;
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  c[0] = p0;
  c[1] = v0;
  c[2] = a0 / 2;
  c[3] = (20 * d - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
  c[4] = (-30 * d + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 *
    T4);
  c[5] = (12 * d - 6 * (v1 + v0) * T - (a0 - a1) * T2) / (2 * T5);
}

double Quintic::pos(double t) const {
  return ((((c[5] * t + c[4]) * t + c[3]) * t + c[2]) * t + c[1]) * t + c[0];
}
double Quintic::vel(double t) const {
  return (((5 * c[5] * t + 4 * c[4]) * t + 3 * c[3]) * t + 2 * c[2]) * t + c[1];
}
double Quintic::acc(double t) const {
  return ((20 * c[5] * t + 12 * c[4]) * t + 6 * c[3]) * t + 2 * c[2];
}

FootTrajectory::Point FootTrajectory::at(double t) const {
  t = std::fmod(t, T);
  if (t < 0) t += T;
  if (t < Tst) return {L / 2 - p.v * t, p.z0};

  const double ts = t - Tst;
  double z = 0.0;

  if (ts < p.Tsw / 2) z = zup.pos(ts);
  else z = zdn.pos(ts - p.Tsw / 2);

  return {cx.pos(ts), z};
}

FootTrajectory::FootTrajectory(double _L, const TrajectoryParams& params) :
    L(_L), p(params) {
  if (p.v == 0 || p.Tsw <= 0) throw std::invalid_argument("Tsw need to be \
    positive, v != 0.");

  Tst = (L / p.v);
  T = Tst + p.Tsw;

  if (Tst < p.Tsw * (1.0 - 1e-9)) throw std::invalid_argument("This robot can't run.");

  cx = Quintic(-L/2, -p.v, 0, L/2, -p.v, 0, p.Tsw);
  zup = Quintic(p.z0, 0, 0, p.z0 + p.h, 0, 0, p.Tsw/2);
  zdn = Quintic(p.z0 + p.h, 0, 0, p.z0, 0, 0, p.Tsw/2);
}
