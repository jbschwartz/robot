#include "third_party/catch.hpp"
#include "matchers/vector.hpp"

#include "../include/transform.hpp"

using rbt::Transform;
using rbt::Vector3;

TEST_CASE("Transform") {
  auto pureTranslate = Transform(Vector3({4, 2, 6}));
  auto pureRotate = Transform(Vector3({1, 0, 0}), 180);

  auto combined = Transform(Vector3({1, 0, 0}), 180, Vector3({4, 2, 6}));

  auto point = Vector3({3, 4, 5});

  SECTION("translates") {
    auto result = pureTranslate(point);
    auto expected = Vector3({7, 6, 11});

    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("rotates") {
    auto result = pureRotate(point);
    auto expected = Vector3({3, -4, -5});

    CHECK_THAT(result, ComponentsEqual(expected));
  }

  SECTION("composition of transforms") {
    auto result = pureRotate(pureTranslate(point));
    auto expected = Vector3({7, -6, -11});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    auto result = t(point);
    auto expected = Vector3({7, -6, -11});

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

      CHECK_THAT(result, ComponentsEqual(expected));
    }

    REQUIRE(Approx(result[0]) == expected[0]);
    REQUIRE(Approx(result[1]) == expected[1]);
    REQUIRE(Approx(result[2]) == expected[2]);
  }

      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }
}
