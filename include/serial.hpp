#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <vector>

#include "joint.hpp"
#include "frame.hpp"
#include "utilities.hpp"
#include "typedefs.hpp"

namespace rbt {

class Serial {
  std::vector<Joint> j;
public:
  Serial(std::vector<Joint> joints) : j(joints) {};

  std::vector<Joint> joints() const;

  // Return the pose of the final joint
  Frame pose(Angles angles) const;
  // Return the poses of all joints
  std::vector<Frame> poses(Angles angles) const;

  inline Real upperArmLength() const { return this->j[1].length(); };
  inline Real foreArmLength() const {
    const auto y = this->j[2].length();
    const auto x = this->j[3].offset();
    return std::sqrt(y * y + x * x);
  };

  inline Real wristLength() const { return this->j[5].offset(); };

  inline Real waistZero() const { return this->j[0].angle(); };

  inline Real shoulderDirection() const { return sign(this->j[0].twist()); };
  inline Real shoulderZero() const { return this->j[1].angle(); };
  inline Real shoulderWristOffset() const { return this->j[1].offset() + this->j[2].offset(); };
  inline Real shoulderZ() const { return this->j[0].offset(); };

  inline Real elbowDirection() const {
    const auto shoulderDirection = this->shoulderDirection();
    return (this->j[1].twist() == PI) ? -shoulderDirection : shoulderDirection;
  };
  inline Real elbowZero() const { return std::atan(this->j[3].offset() / this->j[2].length()); };

  inline std::vector<Vector2> limits() const {
    std::vector<Vector2> limits;
    for(auto joint : this->j) {
      limits.push_back(joint.limits());
    }
    return limits;
  }

};

}

#endif /* __SERIAL_HPP__ */
