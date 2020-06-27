#include "../third_party/catch.hpp"
#include "frame.hpp"
#include "spatial/quaternion.hpp"

using rbt::Frame;
using rbt::Quaternion;

class FrameMatcher : public Catch::MatcherBase<Frame>
{
  Quaternion position;
  Quaternion orientation;

public:
  FrameMatcher(const Frame& a) : position(a.pose().d), orientation(a.pose().r) {};

  virtual bool match(const Frame& b) const override {
    const auto bPosition = b.pose().d;
    const auto bOrientation = b.pose().r;

    // I can probably do better than this... right...?
    return bPosition.r == Approx(position.r) &&
      bPosition.x == Approx(position.x) &&
      bPosition.y == Approx(position.y) &&
      bPosition.z == Approx(position.z) &&
      bOrientation.r == Approx(orientation.r) &&
      bOrientation.x == Approx(orientation.x) &&
      bOrientation.y == Approx(orientation.y) &&
      bOrientation.z == Approx(orientation.z);
  }

  virtual std::string describe() const override {
      std::ostringstream ss;
      return ss.str();
  }
};

inline FrameMatcher FrameEquals(const Frame& a) {
  return FrameMatcher(a);
}
