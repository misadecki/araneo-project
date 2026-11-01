#include "quadruped_kinematics/kinematics.hh"

namespace quadruped {
  const std::array<LegMount, LEG_COUNT> legMounts = {{
    {{0, 0, 0}, 0.0}, //fl
    {{0, 0, 0}, 0.0}, //fr
    {{0, 0, 0}, PI},  //bl
    {{0, 0, 0}, PI}   //br
  }};

  Vector3D bodyToLegFrame(const Vector3D &p, Leg leg) {
    const auto& m = legMounts[leg];
    const double dx = p.x - m.offset.x;
    const double dy = p.y - m.offset.y;
    const double dz = p.z - m.offset.z;
    const double c = std::cos(m.yaw), s = std::sin(m.yaw);

    return {dx * c + dy * s, -dx * s + dy * c, dz};
  }
  
  JointAngles legIK(const Vector3D &p, const LegGeometry &g) {
    const double q1 = std::atan2(p.y, p.x);
    const double r = std::hypot(p.x, p. y);

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

    return {r * std::cos(q.q1), r * std::sin(q.q1), z};
  }
};
