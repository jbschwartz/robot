#include "../include/frame.hpp"
#include "../include/utilities.hpp"
#include <cmath>
#include <algorithm>

namespace rbt {

Frame::Frame() {};

Frame::Frame(const Dual<Quaternion>& pose) : pose(pose) {};

template<>
EulerAngles Frame::euler<Intrinsic::ZYX>() const {
  const auto r = this->pose.r.r;
  const auto x = this->pose.r.x;
  const auto y = this->pose.r.y;
  const auto z = this->pose.r.z;

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
