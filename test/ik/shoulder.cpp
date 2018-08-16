#include "../third_party/catch.hpp"
#include "../matchers/angles.hpp"
#include "../../include/ik.hpp"
#include "../../include/utilities.hpp"
#include "../../include/typedefs.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("solveShoulder") {
  const auto upperArmLength = 10;
  const auto foreArmLength = 5;

  const auto theta = Vector2({ toRadians(10), toRadians(45) });

  const Real x = upperArmLength * std::cos(theta[0]) + foreArmLength * std::cos(theta[0] + theta[1]);
  const Real y = upperArmLength * std::sin(theta[0]) + foreArmLength * std::sin(theta[0] + theta[1]);

  SECTION("returns no solutions") {
    SECTION("for empty elbow angles") {
      const auto result = solveShoulder(x, y, upperArmLength, foreArmLength, Angles());

      REQUIRE(result.empty());
    }
  }

  SECTION("returns one solution in radians") {
    SECTION("for a position on internal workspace boundary") {
      const auto internalBoundary = upperArmLength - foreArmLength;
      const auto elbow = solveElbow(internalBoundary, 0, upperArmLength, foreArmLength);
      const auto result = solveShoulder(internalBoundary, 0, upperArmLength, foreArmLength, elbow);
      const auto expected = Angles({ 0.0f });

      REQUIRE(result.size() == 1);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("for a position on external workspace boundary") {
      const auto fullReach = upperArmLength + foreArmLength;
      const auto elbow = solveElbow(fullReach, 0, upperArmLength, foreArmLength);
      const auto result = solveShoulder(fullReach, 0, upperArmLength, foreArmLength, elbow);
      const auto expected = Angles({ 0.0 });

      REQUIRE(result.size() == 1);
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }

  SECTION("returns two solutions in radians for a position in the workspace") {
    const auto elbow = solveElbow(x, y, upperArmLength, foreArmLength);
    const auto result = solveShoulder(x, y, upperArmLength, foreArmLength, elbow);
    const auto expected = Angles({
      theta[0],
      2 * std::atan2(y, x) - theta[0]
    });

    REQUIRE(result.size() == 2);
    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("returns many solutions (singular) for the origin (i.e. on shoulder axis)") {
    const auto sameLinkLength = upperArmLength;
    const auto elbow = solveElbow(0, 0, sameLinkLength, sameLinkLength);
    const auto result = solveShoulder(0, 0, sameLinkLength, sameLinkLength, elbow);
    const auto expected = Angles({ SINGULAR });

    CHECK_THAT(result, ComponentsEqual(expected));
  }
}
