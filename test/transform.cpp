#include "third_party/catch.hpp"
#include "../include/transform.h"

TEST_CASE("Transform") {
  auto pureTranslate = Transform(Vector3({0, 0, 0}), 0, Vector3({4, 2, 6}));
  auto pureRotate = Transform(Vector3({1, 0, 0}), 180, Vector3());

  auto combined = Transform(Vector3({1, 0, 0}), 180, Vector3({4, 2, 6}));

  auto point = Vector3({3, 4, 5});

  SECTION("translates") {
    auto result = pureTranslate(point);
    auto expected = Vector3({7, 6, 11});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

  SECTION("rotates") {
    auto result = pureRotate(point);
    auto expected = Vector3({3, -4, -5});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

  SECTION("composition of transforms") {
    auto result = pureRotate(pureTranslate(point));
    auto expected = Vector3({7, -6, -11});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

  SECTION("combined translate then rotate") {
    auto result = combined(point);
    auto expected = Vector3({7, -6, -11});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

}
