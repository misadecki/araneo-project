#include "quadruped_gait/trajectory.hh"
#include "quadruped_kinematics/kinematics.hh"
#include <fstream>
#include <cmath>

int main() {
  const auto &g = quadruped::legGeometries[0];
  quadruped::JointAngles qn = {0.0, M_PI};
  const auto nom = quadruped::legFK(qn, g);
  FootTrajectory traj(0.08, 0.10, 0.30, 0.03, nom.z);

  const double T = traj.period();
  const double dt = 0.001;
  const int N = static_cast<int>(2 * T / dt);

  std::ofstream f("traj.csv");
  f << "t,x_ref,z_ref,y_tgt,q1,q2,x_fk,y_fk,z_fk,err,dz\n";

  for (int i = 0; i <= N; ++i) {
    const double t = i * dt;
    const auto p = traj.at(t);
    quadruped::Vector3D vec = {nom.x, nom.y + p.x, p.z};

    auto q = quadruped::legIK(vec, g);
    auto pos = quadruped::legFK(q, g);

    const double err = std::sqrt(std::pow(pos.x - vec.x, 2) + std::pow(pos.y -
                        vec.y, 2) + std::pow(pos.z - vec.z, 2));
    
    const double dz = pos.z - vec.z;

    f << t << ',' << p.x << ',' << p.z << ',' << vec.y << ',' << q.q1 << ',' << q.q2 << ',' <<
    pos.x << ',' << pos.y << ',' << pos.z << ',' << err << ',' << dz << '\n';
  }

  return 0;
}
