#include "third_party/catch.hpp"
#include "matchers/angles.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"
#include "../include/typedefs.hpp"

#include <algorithm>

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("Inverse Kinematics") {
  SECTION("waistAngles") {
    SECTION("with no shoulder offset") {
      const Real angle = 30;
      const Real x = 3 * cos(toRadians(angle));
      const Real y = 3 * sin(toRadians(angle));

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y);
        const auto expected = Angles({
          toRadians(angle),
          toRadians(angle + 180)
        });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("appropriately determines singular position") {
        const auto result = waistAngles(0, 0);
        const auto expected = Angles({ SINGULAR });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }

    SECTION("with shoulder offset") {
      const Real x = 0;
      const Real y = 5;
      const Real offset = 2;

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y, offset);

        // This calculation is a special case when x = 0:
        //            ^ Y
        //            |
        //            X <--- (0, Y)
        //          / |
        //        /   |
        //  -\--- \   |
        // offset  \  |
        //     \    \-| <--- Angle = acos(offset / Y)
        //     _\___ \|
        // -----------X--(0,0)------> X
        const auto expected = Angles({
          std::acos(offset / y),
          std::asin(offset / y) + toRadians(270)
        });

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

}
