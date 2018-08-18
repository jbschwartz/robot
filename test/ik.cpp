#include "third_party/catch.hpp"
#include "matchers/angles.hpp"
#include "matchers/vector.hpp"
#include "robots/abb_irb_120.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"
#include "../include/vector.hpp"
#include "../include/joint.hpp"
#include "../include/typedefs.hpp"
#include "../include/serial.hpp"

#include <algorithm>

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("Inverse Kinematics") {
  SECTION("angles") {
    const auto expected = Angles({ toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45), toRadians(45) });

    const auto frame = ABB_IRB_120.pose(expected);
    const auto result = angles(frame, ABB_IRB_120.joints());

    CHECK_THAT(result.front(), ComponentsEqual(expected));
  }

  SECTION("rsCoordinates") {
    const Real x = 5;
    const Real y = -5;
    const Real z = 10;
    const Real shoulderOffset = 3;
    const Real baseOffset = 4;

    SECTION("calculates RS coordinates") {
      SECTION("with no offsets") {
        const Vector2 result = rsCoordinates(x, y, z, 0, 0);
        const auto expected = Vector2({ 7.07106, 10 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("with a shoulder offset") {
        const auto result = rsCoordinates(x, y, z, shoulderOffset, 0);
        const auto expected = Vector2({ 6.40312, 10 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("with both shoulder and base offsets") {
        const auto result = rsCoordinates(x, y, z, shoulderOffset, baseOffset);
        const auto expected = Vector2({ 6.40312, 6 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }

  SECTION("withinLimits") {
    const Real low = -100;
    const Real high = 212.1;
    const auto limits = Vector2({ low, high });

    SECTION("returns false for angles outside limits") {
      REQUIRE_FALSE(withinLimits(low - 100, limits));
      REQUIRE_FALSE(withinLimits(high + 100, limits));
    }

    SECTION("returns true for angles inside the limits") {
      REQUIRE(withinLimits(low + 10, limits));
      REQUIRE(withinLimits(0, limits));
      REQUIRE(withinLimits(high - 10, limits));
    }

    SECTION("returns true for angles at the limits") {
      REQUIRE(withinLimits(low, limits));
      REQUIRE(withinLimits(high, limits));
    }

    SECTION("returns true for singularities") {
      REQUIRE(withinLimits(SINGULAR, limits));
    }

    SECTION("returns true if limits are infinite") {
      const auto infiniteLimits = Vector2({-INFINITY, INFINITY});

      REQUIRE(withinLimits(low, infiniteLimits));
      REQUIRE(withinLimits(high, infiniteLimits));
      REQUIRE(withinLimits(0, infiniteLimits));
      REQUIRE(withinLimits(SINGULAR, infiniteLimits));
    }

    SECTION("swaps low and high limits") {
      const auto switchedLimits = Vector2({212.1, -100});

      REQUIRE(withinLimits(low, switchedLimits));
      REQUIRE(withinLimits(high, switchedLimits));
      REQUIRE(withinLimits(0, switchedLimits));
      REQUIRE(withinLimits(SINGULAR, switchedLimits));
    }
  }

  SECTION("removeIfBeyondLimits") {
    auto sets = AngleSets({
      AngleSet({ SINGULAR, 200, -100 }),  // Valid
      AngleSet({ SINGULAR, 400, 0 }),     // Invalid
      AngleSet({ 20, SINGULAR, -100 }),   // Valid
      AngleSet({ 300, SINGULAR, -100 }),  // Invalid
      AngleSet({ 20, 200, SINGULAR }),    // Valid
      AngleSet({ 0, -200, SINGULAR }),    // Invalid
      AngleSet({ 20, 200, -120 }),        // Valid
      AngleSet({ 300, 0, -100 }),         // Invalid
      AngleSet({ 20, 20, -20 }),          // Valid
    });

    const auto limits = std::vector<Vector2>({
      Vector2({ -100, 100 }),
      Vector2({ -150, 250 }),
      Vector2({ 200, -120 })
    });

    removeIfBeyondLimits(sets, limits);

    //TODO: Something more thorough...
    REQUIRE(sets.size() == 5);
  }

  SECTION("buildPositionSets") {
    SECTION("needs tests") {
      REQUIRE(true == true);
    }
  }
}
