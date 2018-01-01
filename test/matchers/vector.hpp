#include "../third_party/catch.hpp"
#include "../include/vector.hpp"

using rbt::Vector3;

class VectorMatcher : public Catch::MatcherBase<Vector3>
{
  Vector3 a;
public:
  VectorMatcher(const Vector3& a) : a(a) {};

  virtual bool match(const Vector3& b) const override {
    return a[0] == Approx(b[0]) && a[1] == Approx(b[1]) && a[2] == Approx(b[2]);
  }

  virtual std::string describe() const override {
      std::ostringstream ss;
      ss << "is equal to " << a[0] << " " << a[1] << " " << a[2];
      return ss.str();
  }
};

inline VectorMatcher ComponentsEqual(const Vector3& a) {
  return VectorMatcher(a);
}
