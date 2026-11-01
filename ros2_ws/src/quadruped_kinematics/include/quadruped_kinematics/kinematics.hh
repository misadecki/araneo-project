#ifndef KINEMATICS_HH
#define KINEMATICS_HH

#include <cmath>
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
  };

  struct LegMount {
    Vector3D offset;
    double yaw;
  };

  enum Leg : std::size_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,

    LEG_COUNT
  };

  constexpr double PI = 3.14159265358979323846264338327950288419716939937510;
  extern const std::array<LegMount, LEG_COUNT> legMounts;

  Vector3D bodyToLegFrame(const Vector3D& p, Leg leg);
  JointAngles legIK(const Vector3D& p, const LegGeometry& g);
  Vector3D legFK(const JointAngles& q, const LegGeometry& g);
}; // namespace quadruped

#endif // KINEMATICS_HH
