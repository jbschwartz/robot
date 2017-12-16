#include "third_party/catch.hpp"
#include "../include/dual.h"
#include "../include/quaternion.h"

TEST_CASE("Dual Quaternions") {
  const auto r = Quaternion(1, 2, 3, 4);
  const auto d = Quaternion(0, 1, 2, 3);

  const auto a = Dual<Quaternion>();
  const auto b = Dual<Quaternion>(r, d);

  SECTION("add") {
    const auto c = b + b;
    REQUIRE(c.r == (r + r));
    REQUIRE(c.d == (d + d));
  }

  SECTION("multiply") {
    const auto c = b * b;
    REQUIRE(c.r == (r * r));
    REQUIRE(c.d == (r * d + r * d));
  }
}
