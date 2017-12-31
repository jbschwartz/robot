#include "third_party/catch.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("Inverse Kinematics") {
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
