#include "../third_party/catch.hpp"
#include "../include/ik.hpp"

using rbt::ik::Angles;

class AnglesMatcher : public Catch::MatcherBase<Angles>
{
  Angles a;
public:
  AnglesMatcher(const Angles& a) : a(a) {};

  virtual bool match(const Angles& b) const override {
    auto bElement = b.begin();

    for(const auto& aElement : this->a) {
      if(Approx(aElement) != *bElement++) return false;
    }
    return true;
  }

  virtual std::string describe() const {
      std::ostringstream ss;
      ss << "is equal to ";
      return ss.str();
  }
};

inline AnglesMatcher ComponentsEqual(const Angles& a) {
  return AnglesMatcher(a);
}
