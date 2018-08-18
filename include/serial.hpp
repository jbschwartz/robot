#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <vector>

#include "joint.hpp"
#include "frame.hpp"
#include "typedefs.hpp"

namespace rbt {

class Serial {
  std::vector<Joint> j;
public:
  Serial(std::vector<Joint> joints) : j(joints) {};

  std::vector<Joint> joints() const;

  // Return the pose of the final joint
  Frame pose(Angles angles) const;
  // Return the poses of all joints
  std::vector<Frame> poses(Angles angles) const;
};

}

#endif /* __SERIAL_HPP__ */
