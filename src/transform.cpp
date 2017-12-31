#include "transform.hpp"
#include "utilities.hpp"

#include <cmath>

#ifdef DEBUG
#include <iostream>
#endif

namespace rbt {

Transform::Transform(const Vector3& axis, Real angle, const Vector3& translation) {
  const auto radians = toRadians(angle);
  const auto c = std::cos(radians / static_cast<Real>(2));
  const auto s = std::sin(radians / static_cast<Real>(2));

  const auto r = Quaternion(c, s * axis);
  const auto t = Quaternion(0, translation);

  this->dual = Dual<Quaternion>(r, 0.5 * t * r);
}

Vector3 Transform::operator()(const Vector3& p) const {
  auto d = Dual<Quaternion>(Quaternion(), Quaternion(0, p));
  auto q = this->dual * d * conjugate(this->dual);
  return Vector3({q.d.x, q.d.y, q.d.z});
}

Transform operator*(const Transform& a, const Transform& b) {
  return Transform(a.dual * b.dual);
}

Transform& Transform::operator*=(const Transform& a) {
  this->dual *= a.dual;
  return *this;
}

#ifdef DEBUG
std::ostream& operator<<(std::ostream& os, const Transform& t) {
  os << t.dual;
  return os;
}
#endif

}
