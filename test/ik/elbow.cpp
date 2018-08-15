#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../../include/ik.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("elbowAngles") {
  const auto upperArmLength = 10;
  const auto foreArmLength = 5;

  SECTION("returns no solutions") {
    SECTION("for a point out of reach") {
      const auto outOfReach = upperArmLength + foreArmLength;
      const auto result = elbowAngles(outOfReach, outOfReach, upperArmLength, foreArmLength);

      REQUIRE(result.empty());
    }

    SECTION("for a point too close to reach") {
      const auto outOfReach = 0;
      const auto result = elbowAngles(outOfReach, outOfReach, upperArmLength, foreArmLength);

      REQUIRE(result.empty());
    }
  }

  SECTION("returns one solution in radians for a position on external workspace boundary") {
    const auto fullReach = upperArmLength + foreArmLength;
    const auto result = elbowAngles(fullReach, 0, upperArmLength, foreArmLength);
    const auto expected = Angles({ 0 });

    REQUIRE(result.size() == 1);
    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("returns two solutions in radians") {
    SECTION("for a position on internal workspace boundary") {
      const auto internalBoundary = upperArmLength - foreArmLength;
      const auto result = elbowAngles(internalBoundary, 0, upperArmLength, foreArmLength);
      const auto expected = Angles({ PI, -PI });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("for the origin (i.e. on the shoulder axis)") {
      const auto result = elbowAngles(0, 0, upperArmLength, upperArmLength);
      const auto expected = Angles({ PI, -PI });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("for a position in the workspace") {
      // Construct a 2R manipulator in the plane so the expected result is prescribed
      //    i.e. complete forward kinematics first so we know what to expect from inverse kinematics
      const auto theta = Vector2({ toRadians(10), toRadians(45) });

      const Real x = upperArmLength * std::cos(theta[0]) + foreArmLength * std::cos(theta[0] + theta[1]);
      const Real y = upperArmLength * std::sin(theta[0]) + foreArmLength * std::sin(theta[0] + theta[1]);

      const auto result = elbowAngles(x, y, upperArmLength, foreArmLength);

      const auto expected = Angles({ theta[1], -theta[1] });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }
}
