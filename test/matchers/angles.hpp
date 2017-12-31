#include "../third_party/catch.hpp"
#include "../include/ik.hpp"

using rbt::ik::AngleSets;

// TODO: This should really be order agnostic as order does not matter.
class AngleSetsMatcher : public Catch::MatcherBase<AngleSets>
{
  AngleSets a;
public:
  AngleSetsMatcher(const AngleSets& a) : a(a) {};

  virtual bool match(const AngleSets& b) const override {
    auto bSet = b.begin();

    for(const auto& aSet : this->a) {
      auto bElement = (*bSet++).begin();
      for(const auto& aElement : aSet) {
        if(Approx(aElement) != *bElement++) return false;
      }
    }
    return true;
  }

  virtual std::string describe() const {
      std::ostringstream ss;
      ss << "is equal to ";
      return ss.str();
  }
};

inline AngleSetsMatcher ComponentsEqual(const AngleSets& a) {
  return AngleSetsMatcher(a);
}
