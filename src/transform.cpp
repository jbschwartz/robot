#include "transform.hpp"
#include "utilities.hpp"

#include <cmath>

#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

Transform::Transform(const Vector3& axis, Real angle, const Vector3& translation) {
  const auto radians = toRadians(angle);
  const auto c = std::cos(radians / (Real)2);
  const auto s = std::sin(radians / (Real)2);

  const auto r = Quaternion(c, s * axis);
  const auto t = Quaternion(0, translation / (Real)2);

  this->dual = Dual<Quaternion>(r, 0.5 * r * t);
}

Vector3 Transform::operator()(const Vector3& p) const {
  auto d = Dual<Quaternion>(Quaternion(), 0.5 * Quaternion(0, p));
  auto q = this->dual * d * conjugate(this->dual);
  auto t = 2 * q.d * conjugate(q.r);

  return Vector3({t.x, t.y, t.z});
}

Transform operator*(const Transform& a, const Transform& b) {
  return Transform(a.dual * b.dual);
}

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Transform& t) {
  os << t.dual;
  return os;
}
#endif

}
