#include "../include/serial.hpp"
#include "../include/joint.hpp"
#include "../include/transform.hpp"

#include <algorithm>

namespace rbt {

std::vector<Joint> Serial::joints() const {
  return this->j;
}

Frame Serial::pose(const std::vector<Real>& angles) const {
  auto t = Transform();
  auto angle = angles.begin();

  std::for_each(this->j.begin(), this->j.end(), [&t, &angle](auto& joint) {
    t *= joint.transform(*angle++);
  });

  return Frame(t.dual);
}

std::vector<Frame> Serial::poses(const std::vector<Real>& angles) const {
  auto t = Transform();
  auto angle = angles.begin();
  std::vector<Frame> frames;

  std::for_each(this->j.begin(), this->j.end(), [&t, &angle, &frames](auto& joint) {
    t *= joint.transform(*angle++);
    frames.emplace_back(Frame(t.dual));
  });

  return frames;
}

}
