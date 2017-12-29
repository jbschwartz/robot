#include "third_party/catch.hpp"
#include "matchers/array.hpp"
#include "../include/frame.hpp"
#include "../include/dual.hpp"
#include "../include/quaternion.hpp"

using rbt::Frame;
using rbt::EulerAngles;
using rbt::Dual;
using rbt::Quaternion;
using rbt::Real;
using rbt::Intrinsic;
using rbt::Extrinsic;
using rbt::euler;

TEST_CASE("Frame") {
  const auto f = Frame(Dual<Quaternion>(
    Quaternion(0.353553, -0.353553, 0.853553, 0.146447),
    Quaternion(0, 0, 0, 0)
  ));

  SECTION("converts to") {
    SECTION("intrinsic Euler ZYX angles") {
      const auto result = euler<Intrinsic::ZYX>(f);
      const auto expected = EulerAngles({-135, 45, 180});
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("extrinsic Euler XYZ angles") {
      const auto result = euler<Extrinsic::XYZ>(f);
      const auto expected = EulerAngles({180, 45, -135});
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }
}
