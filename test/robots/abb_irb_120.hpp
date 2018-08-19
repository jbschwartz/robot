#include "../../include/serial.hpp"
#include "../../include/joint.hpp"
#include "../../include/utilities.hpp"
#include "../../include/vector.hpp"

namespace rbt {

const auto ABB_IRB_120 = Serial({
  Joint(toRadians( -90),    0, toRadians(   0),  290, Vector2({ toRadians(-165), toRadians(165) })),
  Joint(toRadians(   0),  270, toRadians( -90),    0, Vector2({ toRadians(-110), toRadians(110) })),
  Joint(toRadians( -90),   70, toRadians(   0),    0, Vector2({ toRadians(-110), toRadians(70)  })),
  Joint(toRadians(  90),    0, toRadians(   0),  302, Vector2({ toRadians(-160), toRadians(160) })),
  Joint(toRadians( -90),    0, toRadians(   0),    0, Vector2({ toRadians(-120), toRadians(120) })),
  Joint(toRadians(   0),    0, toRadians( 180),   72, Vector2({ toRadians(-400), toRadians(400) }))
});

}
