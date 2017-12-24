#include "third_party/catch.hpp"
#include "matchers/vector.hpp"

#include "../include/transform.hpp"

using rbt::Transform;
using rbt::Vector3;

TEST_CASE("Transform") {
  const auto pureTranslate = Transform(Vector3({4, 2, 6}));
  const auto pureRotate = Transform(Vector3({1, 0, 0}), 180);

  const auto point = Vector3({3, 4, 5});

  SECTION("translates a point") {
    const auto result = pureTranslate(point);
    const auto expected = Vector3({7, 6, 11});

    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("rotates a point") {
    const auto result = pureRotate(point);
    const auto expected = Vector3({3, -4, -5});

    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("translates then rotates") {
    const auto expected = Vector3({7, -6, -11});

    SECTION("through composition (operator())") {
      const auto result = pureRotate(pureTranslate(point));

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("by combining two transformations (operator*)") {
      const auto combined = pureRotate * pureTranslate;
      const auto result = combined(point);

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("by transforming one transformation with another (operator*=)") {
      auto combined = pureRotate;
      combined *= pureTranslate;

      const auto result = combined(point);

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("by a-priori combination") {
      const auto combined = Transform(Vector3({1, 0, 0}), 180, Vector3({4, 2, 6}));
      const auto result = combined(point);

      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }
}
