#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <vector>

#include "joint.hpp"
#include "frame.hpp"

namespace rbt {

class Serial {
  std::vector<Joint> j;
public:
  Serial(std::vector<Joint> joints) : j(joints) {};

  std::vector<Joint> joints() const;

  // Return the pose of the final joint
  Frame pose(const std::vector<Real>& angles) const;
  // Return the poses of all joints
  std::vector<Frame> poses(const std::vector<Real>& angles) const;
};

}

#endif /* __SERIAL_HPP__ */
