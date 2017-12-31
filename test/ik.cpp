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

      SECTION("calculates waist angles in radians with no limits") {
        const auto result = waistAngles(x, y);
        const auto expected = Angles({toRadians(angle), toRadians(angle + 180)});

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("calculates waist angles in radians with limits") {
        const auto result = waistAngles(x, y, Vector2({toRadians(angle + 70), toRadians(angle + 70 + 180)}));
        const auto expected = Angles({toRadians(angle + 180)});

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("appropriately determines singular position") {
        const auto result = waistAngles(0, 0);
        const auto expected = Angles({SINGULAR});

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }

  SECTION("removeIfOutsideLimits") {
    auto angles = Angles({-140.5, 212.25, 170, 212.1, 0, 400.40});
    const auto limits = Vector2({-100, 212.1});

    SECTION("prunes angles outside the limits") {
      const auto expected = Angles({170, 212.1, 0});

      removeIfOutsideLimits(angles, limits);

      REQUIRE(angles == expected);
    }

    SECTION("does not remove singularities") {
      angles.push_back(SINGULAR);

      const auto expected = Angles({170, 212.1, 0, SINGULAR});

      removeIfOutsideLimits(angles, limits);

      REQUIRE(angles == expected);
    }

    SECTION("does not remove anything if limits are infinite") {
      const auto equalLimits = Vector2({-INFINITY, INFINITY});

      const auto expected = angles;

      removeIfOutsideLimits(angles, equalLimits);

      REQUIRE(angles == expected);
    }

    SECTION("handles swapped low and high limits") {
      const auto switchedLimits = Vector2({212.1, -100});

      const auto expected = Angles({170, 212.1, 0});

      removeIfOutsideLimits(angles, switchedLimits);

      REQUIRE(angles == expected);
    }

  }
}
