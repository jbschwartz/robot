#include "third_party/catch.hpp"
#include "vector.h"

TEST_CASE("Vector") {
  const auto v1 = Vector3();
  const auto v2 = Vector3({-1, 2, -3});

  SECTION("defaults to zero vector") {
    REQUIRE(v1[0] == 0);
    REQUIRE(v1[1] == 0);
    REQUIRE(v1[2] == 0);
  }

  SECTION("equality operator") {
    REQUIRE(v1 == v1);
    REQUIRE_FALSE(v1 == v2);
  }

  SECTION("scalar multiplication") {
    const Real s = 3;
    REQUIRE(s * v2 == Vector3({-3, 6, -9}));
    REQUIRE(v2 * s == Vector3({-3, 6, -9}));
  }
}
