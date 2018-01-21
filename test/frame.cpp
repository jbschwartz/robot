#include "third_party/catch.hpp"
#include "matchers/array.hpp"
#include "../include/frame.hpp"
#include "../include/dual.hpp"
#include "../include/quaternion.hpp"
#include "../include/utilities.hpp"

using rbt::Frame;
using rbt::EulerAngles;
using rbt::Dual;
using rbt::Quaternion;
using rbt::Real;
using rbt::Intrinsic;
using rbt::Extrinsic;
using rbt::euler;
using rbt::toRadians;

TEST_CASE("Frame") {
  // Rotate around Z 45 degrees, rotation around new Y 135 degrees
  const auto f = Frame(Dual<Quaternion>(
    Quaternion(0.353553, -0.353553, 0.853553, 0.146447),
    Quaternion(0, 0, 0, 0)
  ));

  SECTION("converts to") {
    SECTION("intrinsic Euler ZYX angles") {
      const auto result = euler<Intrinsic::ZYX>(f);
      const auto expected = EulerAngles({toRadians(-135), toRadians(45), toRadians(180)});
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("extrinsic Euler XYZ angles") {
      const auto result = euler<Extrinsic::XYZ>(f);
      const auto expected = EulerAngles({toRadians(180), toRadians(45), toRadians(-135)});
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }
}
