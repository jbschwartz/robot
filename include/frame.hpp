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
  Dual<Quaternion> p;

public:
  Frame();
  Frame(const Dual<Quaternion>& pose);

  Vector3 position() const;
  Quaternion orientation() const;
  Dual<Quaternion> pose() const;
};

template <Intrinsic R>
EulerAngles euler(const Frame& f);

template <Extrinsic R>
EulerAngles euler(const Frame& f) {
  auto intrinsic = euler<static_cast<Intrinsic>(R)>(f);
  std::reverse(intrinsic.begin(), intrinsic.end());
  return intrinsic;
};

}

#endif /* __FRAME_H_ */