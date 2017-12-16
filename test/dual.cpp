#include "third_party/catch.hpp"
#include "../include/dual.h"
#include "../include/quaternion.h"

TEST_CASE("Dual") {
  SECTION("Number") {

  }

  SECTION("Quaternions") {
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

    SECTION("scalar multiply") {
      const Real s = 4;
      const auto c = s * b;
      const auto e = b * s;

      REQUIRE(c.r == s * r);
      REQUIRE(e.r == s * r);

      REQUIRE(c.d == s * d);
      REQUIRE(e.d == s * d);
    }
  }
}
