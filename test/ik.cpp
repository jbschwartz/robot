#include "third_party/catch.hpp"
#include "../include/ik.hpp"

using rbt::Vector2;
using rbt::ik::Angles;
using rbt::ik::removeIfOutsideLimits;
using rbt::ik::SINGULAR;

TEST_CASE("Inverse Kinematics") {
  SECTION("removeIfOutsideLimits") {
    auto angles = Angles({-140.5, 212.25, 170, 0, 400.40});
    const auto limits = Vector2({-100, 212.1});

    SECTION("prunes angles outside the limits") {
      const auto expected = Angles({170, 0});

      removeIfOutsideLimits(angles, limits);

      REQUIRE(angles == expected);
    }

    SECTION("does not remove singularities") {
      angles.push_back(SINGULAR);

      const auto expected = Angles({170, 0, SINGULAR});

      removeIfOutsideLimits(angles, limits);

      REQUIRE(angles == expected);
    }
  }
}
