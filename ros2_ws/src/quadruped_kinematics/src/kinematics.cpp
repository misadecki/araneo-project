#include "quadruped_kinematics/kinematics.hh"
#include <cmath>

namespace quadruped {
  //from URDF file coxa links
  const std::array<LegMount, LEG_COUNT> legMounts = {{
    {{ -0.02987, -0.0401, 0.02825}, -2.0227, -1, -1, -2.9943}, //front left
    {{-0.02987, 0.0401, 0.02825}, 1.6365, -1, 1, -2.9861}, //front right
    {{0.04173, -0.0401, 0.02825}, -0.9243, -1, -1, -2.9861},  //back left
    {{0.04173, 0.0401, 0.02825}, 0.9205, -1, 1, -2.9861}   //back right
  }};

  const std::array<LegGeometry, LEG_COUNT> legGeometries = {{
    // double femur_len;
    // double tibia_len;
    // double femur_ang;
    // double lateral;
    {0.0598, 0.0436, -0.2633, 0.0105},
    {0.0598, 0.0436, -0.2794, -0.0105},
    {0.0598, 0.0436, -0.2794, 0.0096},
    {0.0598, 0.0436, -0.2794, -0.0095}
  }};

  static double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
  }

  JointAngles urdfToModel(const JointAngles& theta, Leg leg) {
    const auto& mount = legMounts[leg];
    return {mount.q1_sign * theta.q1, wrap(mount.q2_zero + mount.q2_sign *
                                                            theta.q2)};
  }

  JointAngles modelToUrdf(const JointAngles& q, Leg leg) {
    const auto& mount = legMounts[leg];
    return {mount.q1_sign * q.q1, mount.q2_sign * wrap(q.q2 - mount.q2_zero)};
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
