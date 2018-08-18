#ifndef __TYPEDEFS_HPP__
#define __TYPEDEFS_HPP__

#include <array>
#include <limits>

namespace rbt {

typedef float Real;
static_assert(std::numeric_limits<Real>::has_infinity, "Type Real must have an infinity value");

typedef std::array<Real, 3> EulerAngles;

}

#endif /* __TYPEDEFS_HPP__ */
