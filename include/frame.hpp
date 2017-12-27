#ifndef __FRAME_H_
#define __FRAME_H_

#include "../include/typedefs.hpp"
#include "../include/dual.hpp"
#include "../include/quaternion.hpp"

#include <algorithm>

namespace rbt {

// Rotations occur about axes of moving coordinate system
enum class Intrinsic {
  ZYX
};

// Rotations occur about axes of fixed coordinate system
// Note Extrinsic must be ordered correctly relative to Intrinsic
enum class Extrinsic {
  XYZ
};

class Frame {
  Dual<Quaternion> pose;

public:
  Frame();
  Frame(const Dual<Quaternion>& pose);

  template <Intrinsic R>
  EulerAngles euler() const;

  template <Extrinsic R>
  EulerAngles euler() const;

  Vector3 position() const;
  Quaternion orientation() const;
};

template <Extrinsic R>
EulerAngles Frame::euler() const {
  auto intrinsic = this->euler<static_cast<Intrinsic>(R)>();
  std::reverse(intrinsic.begin(), intrinsic.end());
  return intrinsic;
};

}

#endif /* __FRAME_H_ */
