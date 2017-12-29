#include "../include/serial.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"

#include <algorithm>

namespace rbt {

Frame Serial::pose(const std::vector<Real>& angles) {
  auto t = Transform();
  auto angle = angles.begin();

  std::for_each(joints.begin(), joints.end(), [&t, &angle](auto& joint) {
    t *= joint.transform(*angle++);
  });

  return Frame(t.dual);
};
// template <std::size_t N>
// std::array<Frame, N> Serial<N>::poses() {
//   return Dual<Quaternion>();
// }
//
}
