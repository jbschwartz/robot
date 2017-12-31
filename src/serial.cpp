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
}

std::vector<Frame> Serial::poses(const std::vector<Real>& angles) {
  auto t = Transform();
  auto angle = angles.begin();
  std::vector<Frame> frames;

  std::for_each(joints.begin(), joints.end(), [&t, &angle, &frames](auto& joint) {
    t *= joint.transform(*angle++);
    frames.emplace_back(Frame(t.dual));
  });

  return frames;
}

}
