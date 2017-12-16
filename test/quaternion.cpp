#include "third_party/catch.hpp"
#include "../include/quaternion.h"

#include <cmath>

TEST_CASE("Quaternions") {
  const auto a = Quaternion();
  const auto b = Quaternion(1, 2, 3, 4);

  SECTION("defaults to identity") {
    REQUIRE(a.r == 1);
    REQUIRE(a.x == 0);
    REQUIRE(a.y == 0);
    REQUIRE(a.z == 0);

    SECTION("identity is multiplicative identity") {
      const auto c = a * b;
      REQUIRE(b == c);
    }
  }

  SECTION("construct in r, x, y, z order") {
    REQUIRE(b.r == 1);
    REQUIRE(b.x == 2);
    REQUIRE(b.y == 3);
    REQUIRE(b.z == 4);
  }

  SECTION("add") {
    const auto c = b + b;
    REQUIRE(Quaternion(2, 4, 6, 8) == c);
  }

  SECTION("multiply") {
    const auto c = b * b;
    REQUIRE(Quaternion(-28, 4, 6, 8) == c);
  }

  SECTION("scalar multiply") {
    const Real s = 4;
    const auto c = s * b;
    const auto d = b * s;

    const auto expected = Quaternion(s * b.r, s * b.x, s * b.y, s * b.z);

    REQUIRE(expected == c);
    REQUIRE(expected == d);
  }

  SECTION("conjugate") {
    const auto c = conjugate(b);
    REQUIRE(Quaternion(1, -2, -3, -4) == c);
  }

  SECTION("norm") {
    REQUIRE(Approx(std::sqrt(30)) == b.norm());
  }

  SECTION("normalize") {
    const auto c = b * b;
    REQUIRE(Quaternion(-14/15.f, 2/15.f, 1/5.f, 4/15.f) == normalize(c));
  }
}
