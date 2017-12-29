#ifndef __SERIAL_H_
#define __SERIAL_H_

#include <vector>

#include "joint.hpp"
#include "frame.hpp"

namespace rbt {

class Serial {
  std::vector<Joint> joints;
public:
  Serial(std::vector<Joint> joints) : joints(joints) {};

  // Return the pose of the final joint
  Frame pose(const std::vector<Real>& angles);
  // Return the poses of all joints
  // std::array<Frame, N> poses() const;
};

}

#endif /* __SERIAL_H_ */
