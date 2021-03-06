template <typename T>
Dual<T>& Dual<T>::operator*=(const Dual<T>& a) {
  const auto r = this->r * a.r;
  const auto d = this->r * a.d + this->d * a.r;

  this->r = r; this->d = d;
  return *this;
}

template <typename T>
Dual<T>& Dual<T>::operator+=(const Dual<T>& a) {
  this->r += a.r; this->d += a.d;
  return *this;
}

template <typename T>
Dual<T>& Dual<T>::operator-=(const Dual<T>& a) {
  this->r -= a.r; this->d -= a.d;
  return *this;
}

template <typename T>
Dual<T> operator+(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r + b.r, a.d + b.d);
}

template <typename T>
Dual<T> operator-(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r - b.r, a.d - b.d);
}

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Dual<T>& b) {
  return Dual<T>(a.r * b.r, a.r * b.d + a.d * b.r);
}

template <typename T>
Dual<T> operator*(const Dual<T>& a, const Real& s) {
  return Dual<T>(s * a.r, s * a.d);
}

template <typename T>
Dual<T> operator*(const Real& s, const Dual<T>& a) {
  return Dual<T>(s * a.r, s * a.d);
}

template <typename T>
Dual<T> operator/(const Dual<T>& a, const Real& s) {
  const Real reciprocal = 1. / s;
  return Dual<T>(reciprocal * a.r, reciprocal * a.d);
}

template <typename T>
bool operator==(const Dual<T>& a, const Dual<T>& b) {
  return a.r == b.r && a.d == b.d;
}

template <typename T>
Dual<T> conjugate(const Dual<T>& a) {
  return Dual<T>(a.r, -a.d);
}

template <typename T>
Dual<T> norm(const Dual<T>& a) {
  return Dual<T>(a * conjugate(a));
}

#ifdef DEBUG
template <typename T>
std::ostream& operator<<(std::ostream& os, const Dual<T>& a) {
  os << a.r << " + " << a.d << "\u03B5";
  return os;
}
#endif
