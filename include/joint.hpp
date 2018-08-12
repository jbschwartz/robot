#ifndef __JOINT_HPP__
#define __JOINT_HPP__

#include "typedefs.hpp"
#include "utilities.hpp"
#include "vector.hpp"

namespace rbt {

class Transform;

class Joint {
  Real alpha, a, theta, d;
  Vector2 lims;
public:
  Joint(const Real& alpha, const Real& a, const Real& theta, const Real& d, const Vector2& limits = Vector2({-360.f, 360.f})) : alpha(alpha), a(a), theta(theta), d(d), lims(limits) {
    lims[0] = toRadians(lims[0]);
    lims[1] = toRadians(lims[1]);
  };
  Transform transform(const Real& theta = 0.0) const;

  Real offset() const { return this->d; };
  Real length() const { return this->a; };
  Vector2 limits() const { return this->lims; };
};

}

#endif /* __JOINT_HPP__ */
