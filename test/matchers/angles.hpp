#include "../third_party/catch.hpp"
#include "../include/ik.hpp"
#include "../include/utilities.hpp"

using rbt::ik::Angles;
using rbt::approxEqual;

class AnglesMatcher : public Catch::MatcherBase<Angles>
{
  Angles a;
public:
  AnglesMatcher(const Angles& a) : a(a) {};

  virtual bool match(const Angles& b) const override {
    if(b.size() != a.size()) return false;

    auto bElement = b.begin();

    for(const auto& aElement : this->a) {
      if(!approxEqual(aElement, *bElement++)) return false;
    }

    return true;
  }

  virtual std::string describe() const override {
      std::ostringstream ss;
      ss << "is not equal to { ";
      for(auto angle : a) {
        ss << angle << " ";
      }
      ss << "}";
      return ss.str();
  }
};

inline AnglesMatcher ComponentsEqual(const Angles& a) {
  return AnglesMatcher(a);
}
