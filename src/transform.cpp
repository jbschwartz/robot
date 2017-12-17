#include "transform.h"
#include "utilities.h"

#include <cmath>

#ifdef DEBUG
#include <iostream>
#endif


Transform::Transform(const Vector3& axis, Real angle, const Vector3& translation) {
  const auto radians = toRadians(angle);
  const auto c = std::cos(radians / (Real)2);
  const auto s = std::sin(radians / (Real)2);

  const auto r = Quaternion(c, s * axis);
  const auto t = Quaternion(0, translation / (Real)2);

  this->dual = Dual<Quaternion>(r, t);
}

Vector3 Transform::operator()(const Vector3& p) {
  auto d = Dual<Quaternion>(Quaternion(), Quaternion(0, p));
  auto q = this->dual * d * conjugate(this->dual);
  return Vector3({q.d.x, q.d.y, q.d.z});
}

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Transform& t) {
  os << t.dual;
  return os;
}
#endif
