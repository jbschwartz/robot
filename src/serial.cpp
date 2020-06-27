#include "serial.hpp"
#include "joint.hpp"
#include "spatial/transform.hpp"

#include <algorithm>

namespace rbt {

std::vector<Joint> Serial::joints() const {
  return this->j;
}

Frame Serial::pose(Angles angles) const {
  auto t = Transform();
  angles.resize(this->j.size());
  auto angle = angles.begin();

  std::for_each(this->j.begin(), this->j.end(), [&t, &angle](auto& joint) {
    t *= joint.transform(*angle++);
  });

  return Frame(t.dual);
}

std::vector<Frame> Serial::poses(Angles angles) const {
  auto t = Transform();
  angles.resize(this->j.size());
  auto angle = angles.begin();
  std::vector<Frame> frames;

  std::for_each(this->j.begin(), this->j.end(), [&t, &angle, &frames](auto& joint) {
    t *= joint.transform(*angle++);
    frames.emplace_back(Frame(t.dual));
  });

  return frames;
}

}
