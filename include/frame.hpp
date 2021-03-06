#ifndef __FRAME_HPP__
#define __FRAME_HPP__

#include "typedefs.hpp"
#include "spatial/dual.hpp"
#include "spatial/quaternion.hpp"

#include <algorithm>

namespace rbt {

// Rotations occur about axes of moving coordinate system
enum class Intrinsic {
  ZYX,
  ZYZ
};

// Rotations occur about axes of fixed coordinate system
// Note Extrinsic must be ordered correctly relative to Intrinsic
enum class Extrinsic {
  XYZ,
  ZYZ
};

class Frame {
  Dual<Quaternion> p;

public:
  Frame();
  Frame(const Dual<Quaternion>& pose);

  Vector3 position() const;
  Quaternion orientation() const;
  Dual<Quaternion> pose() const;

  Vector3 xAxis() const;
  Vector3 yAxis() const;
  Vector3 zAxis() const;
};

template <Intrinsic R>
EulerAngles euler(const Frame& f);

template <Extrinsic R>
EulerAngles euler(const Frame& f) {
  auto intrinsic = euler<static_cast<Intrinsic>(R)>(f);
  std::reverse(intrinsic.begin(), intrinsic.end());
  return intrinsic;
}

}

#endif /* __FRAME_HPP__ */
