#include "third_party/catch.hpp"
#include "matchers/angles.hpp"
#include "matchers/vector.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"
#include "../include/vector.hpp"
#include "../include/joint.hpp"
#include "../include/typedefs.hpp"

#include <algorithm>

using namespace rbt;
using namespace rbt::ik;

TEST_CASE("Inverse Kinematics") {
  SECTION("waistAngles") {
    SECTION("with no shoulder offset") {
      const Real angle = 30;
      const Real x = 3 * cos(toRadians(angle));
      const Real y = 3 * sin(toRadians(angle));

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y);
        const auto expected = Angles({
          toRadians(angle),
          toRadians(angle + 180)
        });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("appropriately determines singular position") {
        const auto result = waistAngles(0, 0);
        const auto expected = Angles({ SINGULAR });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }

    SECTION("with shoulder offset") {
      const Real x = 0;
      const Real y = 5;
      const Real offset = 2;

      SECTION("calculates waist angles in radians") {
        const auto result = waistAngles(x, y, offset);

        // This calculation is a special case when x = 0:
        //            ^ Y
        //            |
        //            X <--- (0, Y)
        //          / |
        //        /   |
        //  -\--- \   |
        // offset  \  |
        //     \    \-| <--- Angle = acos(offset / Y)
        //     _\___ \|
        // -----------X--(0,0)------> X
        const auto expected = Angles({
          std::acos(offset / y),
          std::asin(offset / y) + toRadians(270)
        });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }

  SECTION("elbowAngles") {
    const auto theta = Vector2({ toRadians(10), toRadians(45) });
    const auto l = Vector2({ 10, 10 });

    const Real x = l[0] * std::cos(theta[0]) + l[1] * std::cos(theta[0] + theta[1]);
    const Real y = l[0] * std::sin(theta[0]) + l[1] * std::sin(theta[0] + theta[1]);

    SECTION("calculates elbow angles in radians") {
      const auto result = elbowAngles(x, y, l[0], l[1]);
      const auto expected = Angles({
        theta[1],
        -theta[1]
      });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("returns an empty set for a point out of reach") {
      const auto outOfReach = l[0] + l[1];
      const auto result = elbowAngles(outOfReach, outOfReach, l[0], l[1]);
      const auto expected = Angles();

      REQUIRE(result.size() == 0);
      REQUIRE(result.empty());
    }

    SECTION("returns an empty set for a point too close to reach") {
      const auto result = elbowAngles(0, 0, l[0], l[1] + 3);
      const auto expected = Angles();

      REQUIRE(result.size() == 0);
      REQUIRE(result.empty());
    }

    SECTION("returns two solutions for a position on internal workspace boundary") {
      const auto result = elbowAngles(0, 0, l[0], l[1]);
      const auto expected = Angles({ toRadians(180), toRadians(-180) });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("returns one solution for a position on external workspace boundary") {
      const auto fullReach = l[0] + l[1];
      const auto angle = toRadians(30);
      const auto result = elbowAngles(std::cos(angle) * fullReach, std::sin(angle) * fullReach, l[0], l[1]);
      const auto expected = Angles({ 0 });

      REQUIRE(result.size() == 1);
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }

  SECTION("shoulderAngles") {
    const auto theta = Vector2({ toRadians(10), toRadians(45) });
    const auto l = Vector2({ 10, 10 });

    const Real x = l[0] * std::cos(theta[0]) + l[1] * std::cos(theta[0] + theta[1]);
    const Real y = l[0] * std::sin(theta[0]) + l[1] * std::sin(theta[0] + theta[1]);

    SECTION("returns an empty set for empty elbow angles") {
      const auto result = shoulderAngles(x, y, l[0], l[1], Angles());
      const auto expected = Angles();

      REQUIRE(result.size() == 0);
      REQUIRE(result.empty());
    }

    SECTION("calculates shoulder angles in radians") {
      const auto elbow = elbowAngles(x, y, l[0], l[1]);
      const auto result = shoulderAngles(x, y, l[0], l[1], elbow);
      const auto expected = Angles({
        theta[0],
        2 * std::atan2(y, x) - theta[0]
      });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("returns two solutions for a position on internal workspace boundary") {
      const auto elbow = elbowAngles(3, 0, l[0], l[1] + 3);
      const auto result = shoulderAngles(3, 0, l[0], l[1] + 3, elbow);
      const auto expected = Angles({ toRadians(180), toRadians(-180) });

      REQUIRE(result.size() == 2);
      CHECK_THAT(result, ComponentsEqual(expected));
    }

    SECTION("returns one solution for a position on external workspace boundary") {
      const auto fullReach = l[0] + l[1];
      const auto angle = toRadians(30);
      const auto elbow = elbowAngles(std::cos(angle) * fullReach, std::sin(angle) * fullReach, l[0], l[1]);
      const auto result = shoulderAngles(std::cos(angle) * fullReach, std::sin(angle) * fullReach, l[0], l[1], elbow);
      const auto expected = Angles({ angle });

      REQUIRE(result.size() == 1);
      CHECK_THAT(result, ComponentsEqual(expected));
    }
  }

  SECTION("positionSets") {
    auto j1 = Joint(-90,	0, 		0, 		290);
    auto j2 = Joint(0,   	270,	-90, 	0);
    auto j3 = Joint(90,		-70, 	180,	0);
    auto j4 = Joint(-90, 	0,  	0, 		302);
    auto j5 = Joint(90, 	0, 		0, 		0);
    auto j6 = Joint(0,   	0, 		0,   	72);

    std::vector<Joint> joints = {j1, j2, j3, j4, j5, j6};
    const auto result = positionSets(302, 0, 630, joints);

    SECTION("calculates position solutions to inverse kinematics") {
      const auto expectedSize = 4;
      REQUIRE(result.size() == expectedSize);

      SECTION("needs tests") {
        REQUIRE(true == true);
      }
    }
  }

  SECTION("rsCoordinates") {
    const Real x = 5;
    const Real y = -5;
    const Real z = 10;
    const Real shoulderOffset = 3;
    const Real baseOffset = 4;

    SECTION("calculates RS coordinates") {
      SECTION("with no offsets") {
        const Vector2 result = rsCoordinates(x, y, z, 0, 0);
        const auto expected = Vector2({ 7.07106, 10 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("with a shoulder offset") {
        const auto result = rsCoordinates(x, y, z, shoulderOffset, 0);
        const auto expected = Vector2({ 6.40312, 10 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }

      SECTION("with both shoulder and base offsets") {
        const auto result = rsCoordinates(x, y, z, shoulderOffset, baseOffset);
        const auto expected = Vector2({ 6.40312, 6 });

        CHECK_THAT(result, ComponentsEqual(expected));
      }
    }
  }

  SECTION("withinLimits") {
    const Real low = -100;
    const Real high = 212.1;
    const auto limits = Vector2({ low, high });

    SECTION("returns false for angles outside limits") {
      REQUIRE_FALSE(withinLimits(low - 100, limits));
      REQUIRE_FALSE(withinLimits(high + 100, limits));
    }

    SECTION("returns true for angles inside the limits") {
      REQUIRE(withinLimits(low + 10, limits));
      REQUIRE(withinLimits(0, limits));
      REQUIRE(withinLimits(high - 10, limits));
    }

    SECTION("returns true for angles at the limits") {
      REQUIRE(withinLimits(low, limits));
      REQUIRE(withinLimits(high, limits));
    }

    SECTION("returns true for singularities") {
      REQUIRE(withinLimits(SINGULAR, limits));
    }

    SECTION("returns true if limits are infinite") {
      const auto infiniteLimits = Vector2({-INFINITY, INFINITY});

      REQUIRE(withinLimits(low, infiniteLimits));
      REQUIRE(withinLimits(high, infiniteLimits));
      REQUIRE(withinLimits(0, infiniteLimits));
      REQUIRE(withinLimits(SINGULAR, infiniteLimits));
    }

    SECTION("swaps low and high limits") {
      const auto switchedLimits = Vector2({212.1, -100});

      REQUIRE(withinLimits(low, switchedLimits));
      REQUIRE(withinLimits(high, switchedLimits));
      REQUIRE(withinLimits(0, switchedLimits));
      REQUIRE(withinLimits(SINGULAR, switchedLimits));
    }
  }

  SECTION("removeIfBeyondLimits") {
    auto sets = AngleSets({
      AngleSet({ SINGULAR, 200, -100 }),  // Valid
      AngleSet({ SINGULAR, 400, 0 }),     // Invalid
      AngleSet({ 20, SINGULAR, -100 }),   // Valid
      AngleSet({ 300, SINGULAR, -100 }),  // Invalid
      AngleSet({ 20, 200, SINGULAR }),    // Valid
      AngleSet({ 0, -200, SINGULAR }),    // Invalid
      AngleSet({ 20, 200, -120 }),        // Valid
      AngleSet({ 300, 0, -100 }),         // Invalid
      AngleSet({ 20, 20, -20 }),          // Valid
    });

    const auto limits = std::vector<Vector2>({
      Vector2({ -100, 100 }),
      Vector2({ -150, 250 }),
      Vector2({ 200, -120 })
    });

    removeIfBeyondLimits(sets, limits);

    //TODO: Something more thorough...
    REQUIRE(sets.size() == 5);
  }

  SECTION("buildPositionSets") {
    SECTION("needs tests") {
      REQUIRE(true == true);
    }
  }
}
