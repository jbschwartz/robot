#ifndef __SERIAL_H_
#define __SERIAL_H_

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
  Frame pose(const std::vector<Real>& angles);
  // Return the poses of all joints
  std::vector<Frame> poses(const std::vector<Real>& angles);
};

}

#endif /* __SERIAL_H_ */
