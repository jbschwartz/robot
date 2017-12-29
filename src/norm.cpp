#include "../include/norm.hpp"

namespace rbt {

Real norm(const Quaternion& a) {
  return std::sqrt(
    a.r * a.r +
    a.x * a.x +
    a.y * a.y +
    a.z * a.z
  );
}

Real norm(const Dual<Quaternion>& a) {
  return norm(a.r);
}

}
