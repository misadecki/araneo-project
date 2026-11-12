#ifndef KINEMATICS_HH
#define KINEMATICS_HH

#include <array>

namespace quadruped {
  struct Vector3D {
    double x = 0;
    double y = 0;
    double z = 0;
  };
  
  struct JointAngles {
    double q1 = 0;
    double q2 = 0;
  };

  struct LegGeometry {
    double femur_len;
    double tibia_len;
    double femur_ang;
    double lateral;
  };

  struct LegMount {
    Vector3D offset;
    double yaw;
    double q1_sign;
    double q2_sign;
    double q2_zero;
  };
  
  struct LegConfig {
    LegMount mount;
    LegGeometry geom;
  };

  enum Leg : std::size_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,

    LEG_COUNT
  };

  using RobotConfig = std::array<LegConfig, LEG_COUNT>;

  constexpr double PI = 3.14159265358979323846264338327950288419716939937510;

  JointAngles urdfToModel(const JointAngles& theta, const LegMount& m);
  JointAngles modelToUrdf(const JointAngles& q, const LegMount& m);
  Vector3D bodyToLegFrame(const Vector3D& p, const LegMount& m);
  Vector3D legToBodyFrame(const Vector3D& p, const LegMount& m);
  JointAngles legIK(const Vector3D& p, const LegGeometry& g);
  Vector3D legFK(const JointAngles& q, const LegGeometry& g);
}; // namespace quadruped

#endif // KINEMATICS_HH
