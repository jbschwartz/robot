#include "third_party/catch.hpp"
#include "../include/dual.h"
#include "../include/quaternion.h"

TEST_CASE("Dual Quaternions") {
  auto r = Quaternion(1, 2, 3, 4);
  auto d = Quaternion(0, 1, 2, 3);

  auto a = Dual<Quaternion>();
  auto b = Dual<Quaternion>(r, d);

  SECTION("add") {
    auto c = b + b;
    REQUIRE(c.r == (r + r));
    REQUIRE(c.d == (d + d));
  }

  SECTION("multiply") {
    auto c = b * b;
    REQUIRE(c.r == (r * r));
    REQUIRE(c.d == (d * d + d * d));
  }
}
