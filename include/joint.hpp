#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include "typedefs.hpp"
#include "utilities.hpp"
#include "vector.hpp"

namespace rbt {

class Transform;

class Joint {
public:
  Real alpha, a, theta, d;
  Vector2 limits;
  Joint(const Real& alpha, const Real& a, const Real& theta, const Real& d, Vector2 limits = Vector2({-360.f, 360.f})) : alpha(alpha), a(a), theta(theta), d(d), limits(limits) {
    limits[0] = toRadians(limits[0]);
    limits[1] = toRadians(limits[1]);
  };
  Transform transform(const Real& theta = 0.0) const;
};

}

#endif /* __JOINT_HPP__ */
