#ifndef __DUAL_H__
#define __DUAL_H__

template <typename T>
class Dual {
public:
  T r, d;
  Dual() : r(T()), d(T()) {}
  Dual(const T& r, const T& d) : r(r), d(d) {};
};

template <typename T>
Dual<T> operator+(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r + b.r, a.d + b.d);
};

#endif /* __DUAL_H__ */
