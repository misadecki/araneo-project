#include "quadruped_kinematics/kinematics.hh"
#include <cmath>

namespace quadruped {
  static double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }

  JointAngles urdfToModel(const JointAngles& theta, const LegMount& m) {
    return {m.q1_sign * theta.q1, wrap(m.q2_zero + m.q2_sign *
                                                            theta.q2)};
  }

  JointAngles modelToUrdf(const JointAngles& q, const LegMount& m) {
    return {m.q1_sign * q.q1, m.q2_sign * wrap(q.q2 - m.q2_zero)};
  }

  Vector3D bodyToLegFrame(const Vector3D &p, const LegMount& m) {
    const double dx = p.x - m.offset.x;
    const double dy = p.y - m.offset.y;
    const double dz = p.z - m.offset.z;
    const double c = std::cos(m.yaw), s = std::sin(m.yaw);

    return {dx * c + dy * s, -dx * s + dy * c, dz};
  }
  
  Vector3D legToBodyFrame(const Vector3D& p, const LegMount& m) {
    const double c = std::cos(m.yaw), s = std::sin(m.yaw);
    return {p.x * c - p.y * s + m.offset.x,
            p.x * s + p.y * c + m.offset.y,
            p.z + m.offset.z};
  }
  
  JointAngles legIK(const Vector3D &p, const LegGeometry &g) {
    const double p_xy_len = p.x * p.x + p.y * p.y;
    const double r = std::sqrt(std::max(p_xy_len - g.lateral * g.lateral, 0.0));
    const double q1 = std::atan2(p.y, p.x) - std::atan2(g.lateral, r);

    const double knee_r = g.femur_len * std::cos(g.femur_ang);
    const double knee_z = g.femur_len * std::sin(g.femur_ang);

    double dr = r - knee_r;
    double dz = p.z - knee_z;
    const double dist = std::hypot(dr, dz);

    if (dist > g.tibia_len && dist > 1e-9) {
      const double scale = g.tibia_len / dist;
      dr *= scale;
      dz *= scale;
    }
    
    return {q1, std::atan2(dr, dz)};
  }

  Vector3D legFK(const JointAngles &q, const LegGeometry &g) {
    const double knee_r = g.femur_len * std::cos(g.femur_ang);
    const double knee_z = g.femur_len * std::sin(g.femur_ang);

    const double r = knee_r + g.tibia_len * std::sin(q.q2);
    const double z = knee_z + g.tibia_len * std::cos(q.q2);

    return {r * std::cos(q.q1) - g.lateral * std::sin(q.q1), r * std::sin(q.q1)
            + g.lateral * std::cos(q.q1), z};
  }
};
