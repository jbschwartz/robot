#include "third_party/catch.hpp"
#include "matchers/array.hpp"
#include "matchers/vector.hpp"
#include "../include/frame.hpp"
#include "../include/dual.hpp"
#include "../include/quaternion.hpp"
#include "../include/utilities.hpp"
#include "../include/vector.hpp"
#include "../include/transform.hpp"

using rbt::Frame;
using rbt::EulerAngles;
using rbt::Dual;
using rbt::Quaternion;
using rbt::Real;
using rbt::Intrinsic;
using rbt::Extrinsic;
using rbt::euler;
using rbt::toRadians;
using rbt::Vector3;
using rbt::Transform;

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

    SECTION("intrinsic Euler ZYZ angles") {
      auto result = euler<Intrinsic::ZYZ>(f);
      const auto expected = EulerAngles({toRadians(45), toRadians(135), toRadians(0)});

      // TODO: Alter the matcher so that it's not necessary to round to 5 decimal places
      std::transform(result.begin(), result.end(), result.begin(), [](auto angle) {
        return std::round(angle * 10000) / 10000;
      });

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("extrinsic Euler ZYZ angles") {
      auto result = euler<Extrinsic::ZYZ>(f);
      const auto expected = EulerAngles({toRadians(0), toRadians(135), toRadians(45)});

      // TODO: Alter the matcher so that it's not necessary to round to 5 decimal places
      std::transform(result.begin(), result.end(), result.begin(), [](auto angle) {
        return std::round(angle * 10000) / 10000;
      });

      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }

  SECTION("Axis functions") {
    const auto frame = Frame(Dual<Quaternion>(
      Quaternion(0.92388, 0, 0, 0.38268),
      Quaternion(0, 0, 0, 0)
    ));
    const auto t = Transform(Vector3({ 0, 0, 1 }), 45);

    SECTION("xAxis") {
      const auto result = frame.xAxis();
      const auto expected = t(Vector3({ 1, 0, 0 }));

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("yAxis") {
      const auto result = frame.yAxis();
      const auto expected = t(Vector3({ 0, 1, 0 }));

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("zAxis") {
      const auto result = frame.zAxis();
      const auto expected = t(Vector3({ 0, 0, 1 }));

      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }


}
