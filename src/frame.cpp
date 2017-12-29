#include "../include/frame.hpp"
#include "../include/utilities.hpp"
#include "../include/vector.hpp"
#include <cmath>
#include <algorithm>

namespace rbt {

Frame::Frame() {};

Frame::Frame(const Dual<Quaternion>& pose) : p(pose) {};

Vector3 Frame::position() const {
  auto t = 2 * this->p.d * conjugate(this->p.r);

  return Vector3({t.x, t.y, t.z});
};

Quaternion Frame::orientation() const {
  return this->p.r;
}

Dual<Quaternion> Frame::pose() const {
  return this->p;
}

template<>
EulerAngles euler<Intrinsic::ZYX>(const Frame& f) {
  const auto orientation = f.orientation();
  const auto r = orientation.r;
  const auto x = orientation.x;
  const auto y = orientation.y;
  const auto z = orientation.z;

  const auto rSq = r * r;
  const auto xSq = x * x;
  const auto ySq = y * y;
  const auto zSq = z * z;

  const auto X = toDegrees(std::atan2(2 * (y * z + r * x), rSq - xSq - ySq + zSq));
  const auto Y = toDegrees(std::asin(-2 * (x * z - r * y)));
  const auto Z = toDegrees(std::atan2(2 * (x * y + r * z), rSq + xSq - ySq - zSq));

  return {Z, Y, X};
}

}
