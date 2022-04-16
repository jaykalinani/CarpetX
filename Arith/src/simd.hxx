#ifndef SIMD_HXX
#define SIMD_HXX

#include "defs.hxx"

#include <nsimd/nsimd-all.hpp>
#undef vec // This should arguably not be defined in C++

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <utility>
#include <type_traits>

namespace Arith {
using namespace std;

template <typename T> struct simd;
template <typename T> struct simdl;

namespace detail {
struct reinterpret32 {
  typedef i32 int_type;
  typedef u32 unsigned_type;
  typedef f32 float_type;
};
struct reinterpret64 {
  typedef i64 int_type;
  typedef u64 unsigned_type;
  typedef f64 float_type;
};

template <typename T> struct reinterpret;
template <> struct reinterpret<i32> : reinterpret32 {};
template <> struct reinterpret<u32> : reinterpret32 {};
template <> struct reinterpret<f32> : reinterpret32 {};
template <> struct reinterpret<i64> : reinterpret64 {};
template <> struct reinterpret<u64> : reinterpret64 {};
template <> struct reinterpret<f64> : reinterpret64 {};

template <typename T> using int_type = typename reinterpret<T>::int_type;
template <typename T>
using unsigned_type = typename reinterpret<T>::unsigned_type;
template <typename T> using float_type = typename reinterpret<T>::float_type;
} // namespace detail

////////////////////////////////////////////////////////////////////////////////

// A SIMD vector, holding elements of type `T`. `T` can be a
// floating-point or an integer type. To represent logical values
// (i.e. booleans), see `simdl` below.

// This class is based on NSIMD
// <https://github.com/agenium-scale/nsimd>. The NSIMD library is very
// close to what we need; with a few bugs corrected or features added,
// this wrapper class here might be avoided.

template <typename T> struct simd {
  typedef T value_type;
#ifndef SIMD_CPU
  nsimd::pack<T> elts;
#else
  T elts;
#endif

  constexpr simd(const simd &) = default;
  constexpr simd(simd &&) = default;
  constexpr simd &operator=(const simd &) = default;
  constexpr simd &operator=(simd &&) = default;

  constexpr ARITH_DEVICE ARITH_HOST simd() {}
  constexpr ARITH_DEVICE ARITH_HOST simd(const T &a) : elts(a) {}
#ifndef SIMD_CPU
  template <typename U = T,
            enable_if_t<!is_same_v<nsimd::pack<T>, T> > * = nullptr>
  constexpr ARITH_DEVICE ARITH_HOST simd(const nsimd::pack<T> &elts)
      : elts(elts) {}
#endif

  constexpr ARITH_DEVICE ARITH_HOST std::size_t size() const {
#ifndef SIMD_CPU
    return sizeof(nsimd::pack<T>) / sizeof(T);
#else
    return 1;
#endif
  }

  constexpr ARITH_DEVICE ARITH_HOST T operator[](const std::ptrdiff_t n) const {
    constexpr size_t vsize = tuple_size_v<simd>;
#ifdef CCTK_DEBUG
    assert(n >= 0 && n < int(vsize));
#endif
    T xarr[vsize];
    storeu(xarr, *this);
    return xarr[n];
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator!(const simd &x) {
    return !x.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator~(const simd &x) {
    return ~x.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd operator+(const simd &x) {
    return +x.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator-(const simd &x) {
    return -x.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd operator&(const simd &x,
                                                          const simd &y) {
    return x.elts & y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator|(const simd &x,
                                                          const simd &y) {
    return x.elts | y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator^(const simd &x,
                                                          const simd &y) {
    return x.elts ^ y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator+(const simd &x,
                                                          const simd &y) {
    return x.elts + y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator-(const simd &x,
                                                          const simd &y) {
    return x.elts - y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator*(const simd &x,
                                                          const simd &y) {
    return x.elts * y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator/(const simd &x,
                                                          const simd &y) {
    return x.elts / y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator%(const simd &x,
                                                          const simd &y) {
    return x.elts % y.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd operator&(const T &a,
                                                          const simd &y) {
    return a & y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator|(const T &a,
                                                          const simd &y) {
    return a | y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator^(const T &a,
                                                          const simd &y) {
    return a ^ y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator+(const T &a,
                                                          const simd &y) {
    return a + y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator-(const T &a,
                                                          const simd &y) {
    return a - y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator*(const T &a,
                                                          const simd &y) {
    return a * y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator/(const T &a,
                                                          const simd &y) {
    return a / y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator%(const T &a,
                                                          const simd &y) {
    return a % y.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd operator&(const simd &x,
                                                          const T &b) {
    return x.elts & b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator|(const simd &x,
                                                          const T &b) {
    return x.elts | b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator^(const simd &x,
                                                          const T &b) {
    return x.elts ^ b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator+(const simd &x,
                                                          const T &b) {
    return x.elts + b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator-(const simd &x,
                                                          const T &b) {
    return x.elts - b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator*(const simd &x,
                                                          const T &b) {
    return x.elts * b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator/(const simd &x,
                                                          const T &b) {
    return x.elts / b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd operator%(const simd &x,
                                                          const T &b) {
    return x.elts % b;
  }

  constexpr ARITH_DEVICE ARITH_HOST simd &operator&=(const simd &x) {
    return *this = *this & x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator|=(const simd &x) {
    return *this = *this | x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator^=(const simd &x) {
    return *this = *this ^ x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator+=(const simd &x) {
    return *this = *this + x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator-=(const simd &x) {
    return *this = *this - x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator*=(const simd &x) {
    return *this = *this * x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator/=(const simd &x) {
    return *this = *this / x;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator%=(const simd &x) {
    return *this = *this % x;
  }

  constexpr ARITH_DEVICE ARITH_HOST simd &operator&=(const T &a) {
    return *this = *this & a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator|=(const T &a) {
    return *this = *this | a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator^=(const T &a) {
    return *this = *this ^ a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator+=(const T &a) {
    return *this = *this + a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator-=(const T &a) {
    return *this = *this - a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator*=(const T &a) {
    return *this = *this * a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator/=(const T &a) {
    return *this = *this / a;
  }
  constexpr ARITH_DEVICE ARITH_HOST simd &operator%=(const T &a) {
    return *this = *this % a;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator==(const simd &x,
                                                               const simd &y) {
    return x.elts == y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator!=(const simd &x,
                                                               const simd &y) {
    return x.elts != y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<(const simd &x,
                                                              const simd &y) {
    return x.elts < y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>(const simd &x,
                                                              const simd &y) {
    return x.elts > y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<=(const simd &x,
                                                               const simd &y) {
    return x.elts <= y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>=(const simd &x,
                                                               const simd &y) {
    return x.elts >= y.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator==(const T &a,
                                                               const simd &y) {
    return a == y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator!=(const T &a,
                                                               const simd &y) {
    return a != y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<(const T &a,
                                                              const simd &y) {
    return a < y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>(const T &a,
                                                              const simd &y) {
    return a > y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<=(const T &a,
                                                               const simd &y) {
    return a <= y.elts;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>=(const T &a,
                                                               const simd &y) {
    return a >= y.elts;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator==(const simd &x,
                                                               const T &b) {
    return x.elts == b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator!=(const simd &x,
                                                               const T &b) {
    return x.elts != b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<(const simd &x,
                                                              const T &b) {
    return x.elts < b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>(const simd &x,
                                                              const T &b) {
    return x.elts > b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator<=(const simd &x,
                                                               const T &b) {
    return x.elts <= b;
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> operator>=(const simd &x,
                                                               const T &b) {
    return x.elts >= b;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd abs(const simd &x) {
    using std::abs;
    return abs(x.elts);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd andnot(const simd &x,
                                                       const simd &y) {
#ifndef SIMD_CPU
    return andnotb(x.elts, y.elts);
#else
    return x & ~y;
#endif
  }

  friend constexpr ARITH_DEVICE ARITH_HOST bool allisfinite(const simd &x) {
    using std::isfinite;
    return all(isfinite(x));
  }

  friend constexpr ARITH_DEVICE ARITH_HOST bool anyisnan(const simd &x) {
    using std::isnan;
    return any(isnan(x));
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd copysign(const simd &x,
                                                         const simd &y) {
#ifndef SIMD_CPU
    typedef detail::unsigned_type<T> U;
    const T signmask =
        nsimd::scalar_reinterpret(T{}, U(1) << (8 * sizeof(U) - 1));
    return andnot(x, signmask) | (y & signmask);
#else
    using std::copysign;
    return copysign(x.elts, y.elts);
#endif
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd fabs(const simd &x) {
    using std::abs;
    return abs(x.elts);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd flipsign(const simd &x,
                                                         const simd &y) {
#ifndef SIMD_CPU
    typedef detail::unsigned_type<T> U;
    const T signmask =
        nsimd::scalar_reinterpret(T{}, U(1) << (8 * sizeof(U) - 1));
    return x ^ (y & signmask);
#else
    using std::copysign;
    return copysign(1, y) * x;
#endif
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd fmax(const simd &x,
                                                     const simd &y) {
    using std::max;
    return max(x.elts, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd fmax(const T &a,
                                                     const simd &y) {
    using std::max;
    return max(a, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd fmax(const simd &x,
                                                     const T &b) {
    using std::max;
    return max(x.elts, b);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd fmin(const simd &x,
                                                     const simd &y) {
    using std::min;
    return min(x.elts, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd fmin(const T &a,
                                                     const simd &y) {
    using std::min;
    return min(a, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd fmin(const simd &x,
                                                     const T &b) {
    using std::min;
    return min(x.elts, b);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> isfinite(const simd &x) {
    // using std::isfinite;
    // return isfinite(x.elts);
    return !isnan(x) && !isinf(x);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> isinf(const simd &x) {
    // using std::isinf;
    // return isinf(x.elts);
    return fabs(x) == numeric_limits<T>::infinity();
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> isnan(const simd &x) {
    // using std::isnan;
    // return isnan(x.elts);
    return x != x;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd max(const simd &x,
                                                    const simd &y) {
    using std::max;
    return max(x.elts, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd max(const simd &x, const T &b) {
    using std::max;
    return max(x.elts, b);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd max(const T &a, const simd &y) {
    using std::max;
    return max(a, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd
  max(std::initializer_list<simd> xs) {
    using std::max;
    simd r = simd(-std::numeric_limits<T>::infinity());
    for (const auto x : xs)
      r = max(r, x);
    return r;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd min(const simd &x,
                                                    const simd &y) {
    using std::min;
    return min(x.elts, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd min(const T &a, const simd &y) {
    using std::min;
    return min(a, y.elts);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd min(const simd &x, const T &b) {
    using std::min;
    return min(x.elts, b);
  }
  friend constexpr ARITH_DEVICE ARITH_HOST simd
  min(std::initializer_list<simd> xs) {
    using std::min;
    simd r = simd(std::numeric_limits<T>::infinity());
    for (const auto x : xs)
      r = min(r, x);
    return r;
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> signbit(const simd &x) {
#ifndef SIMD_CPU
    typedef detail::unsigned_type<T> U;
    const T signmask =
        nsimd::scalar_reinterpret(T{}, U(1) << (8 * sizeof(U) - 1));
    return to_logical(x & signmask);
#else
    using std::signbit;
    return signbit(x.elts);
#endif
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simd sqrt(const simd &x) {
    using std::sqrt;
    return sqrt(x.elts);
  }

  friend constexpr ARITH_DEVICE ARITH_HOST simdl<T> to_logical(const simd &x) {
    return to_logical(x.elts);
  }

  friend ARITH_DEVICE ARITH_HOST void storea(T *ptr, const simd &x) {
#ifndef SIMD_CPU
    storea(ptr, x.elts);
#else
    *ptr = x.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST void storeu(T *ptr, const simd &x) {
#ifndef SIMD_CPU
    storeu(ptr, x.elts);
#else
    *ptr = x.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST void mask_storea(const simdl<T> &mask, T *ptr,
                                                  const simd &x) {
#ifndef SIMD_CPU
    mask_storea(mask.elts, ptr, x.elts);
#else
    if (mask.elts)
      *ptr = x.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST void mask_storeu(const simdl<T> &mask, T *ptr,
                                                  const simd &x) {
#ifndef SIMD_CPU
    mask_storeu(mask.elts, ptr, x.elts);
#else
    if (mask.elts)
      *ptr = x.elts;
#endif
  }

  friend ostream &operator<<(ostream &os, const simd &x) {
    os << "Ⓢ[";
    constexpr size_t vsize = tuple_size_v<simd>;
    for (size_t n = 0; n < vsize; ++n) {
      if (n != 0)
        os << ",";
      os << x[n];
    }
    os << "]";
    return os;
  }
};

} // namespace Arith
namespace std {
template <typename T>
struct tuple_size<Arith::simd<T> >
    : std::integral_constant<std::size_t, sizeof(Arith::simd<T>) / sizeof(T)> {
};
} // namespace std
namespace Arith {

template <typename T> struct zero<simd<T> > {
  typedef simd<T> value_type;
  // static constexpr value_type value = simd<T>(zero_v<T>);
  constexpr ARITH_INLINE operator value_type() const {
    return simd<T>(zero<T>());
  }
  constexpr ARITH_INLINE value_type operator()() const {
    return simd<T>(zero<T>());
  }
};

template <typename T> struct one<simd<T> > {
  typedef simd<T> value_type;
  // static constexpr value_type value = simd<T>(one_v<T>);
  constexpr ARITH_INLINE operator value_type() const {
    return simd<T>(one<T>());
  }
  constexpr ARITH_INLINE value_type operator()() const {
    return simd<T>(one<T>());
  }
};

template <typename T> struct nan<simd<T> > {
  typedef simd<T> value_type;
  // static constexpr value_type value = simd<T>(nan_v<T>);
  constexpr ARITH_INLINE operator value_type() const {
    return simd<T>(nan<T>());
  }
  constexpr ARITH_INLINE value_type operator()() const {
    return simd<T>(nan<T>());
  }
};

template <typename VT, typename T = typename VT::value_type>
ARITH_DEVICE ARITH_HOST inline simd<T> iota() {
#ifndef SIMD_CPU
  return nsimd::iota<nsimd::pack<T> >();
#else
  return 0;
#endif
}

template <typename VT, typename T = typename VT::value_type>
ARITH_DEVICE ARITH_HOST inline simdl<T> mask_for_loop_tail(const int i,
                                                           const int n) {
#ifndef SIMD_CPU
  return nsimd::mask_for_loop_tail<nsimd::packl<T> >(i, n);
#else
  return i < n;
#endif
}

template <typename VT, typename T = typename VT::value_type>
ARITH_DEVICE ARITH_HOST inline simd<T> loada(const T *ptr) {
#ifndef SIMD_CPU
  return nsimd::loada<nsimd::pack<T> >(ptr);
#else
  return *ptr;
#endif
}

template <typename VT, typename T = typename VT::value_type>
ARITH_DEVICE ARITH_HOST inline simd<T> loadu(const T *ptr) {
#ifndef SIMD_CPU
  return nsimd::loadu<nsimd::pack<T> >(ptr);
#else
  return *ptr;
#endif
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T> maskz_loada(const simdl<T> &mask,
                                                   const T *ptr) {
#ifndef SIMD_CPU
  return nsimd::maskz_loada(mask.elts, ptr);
#else
  return mask.elts ? *ptr : 0;
#endif
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T> maskz_loadu(const simdl<T> &mask,
                                                   const T *ptr) {
#ifndef SIMD_CPU
  return nsimd::maskz_loadu(mask.elts, ptr);
#else
  return mask.elts ? *ptr : 0;
#endif
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T>
masko_loada(const simdl<T> &mask, const T *ptr, const simd<T> &other) {
#ifndef SIMD_CPU
  return masko_loada(mask.elts, ptr, other.elts);
#else
  return mask.elts ? *ptr : other.elts;
#endif
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T>
masko_loadu(const simdl<T> &mask, const T *ptr, const simd<T> &other) {
#ifndef SIMD_CPU
  return masko_loadu(mask.elts, ptr, other.elts);
#else
  return mask.elts ? *ptr : other.elts;
#endif
}

template <typename T, typename U,
          enable_if_t<is_convertible_v<T, U> > * = nullptr>
ARITH_DEVICE ARITH_HOST inline simd<T>
masko_loada(const simdl<T> &mask, const T *ptr, const U &other) {
#ifndef SIMD_CPU
  return masko_loada(mask, ptr, simd<T>(other));
#else
  return mask.elts ? *ptr : other;
#endif
}

template <typename T, typename U,
          enable_if_t<is_convertible_v<T, U> > * = nullptr>
ARITH_DEVICE ARITH_HOST inline simd<T>
masko_loadu(const simdl<T> &mask, const T *ptr, const U &other) {
#ifndef SIMD_CPU
  return masko_loadu(mask, ptr, simd<T>(other));
#else
  return mask.elts ? *ptr : other;
#endif
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T> cbrt(const simd<T> &x) {
  constexpr size_t vsize = tuple_size_v<simd<T> >;
  alignas(alignof(simd<T>)) T xarr[vsize];
  storea(xarr, x);
  alignas(alignof(simd<T>)) T yarr[vsize];
  for (std::size_t n = 0; n < x.size(); ++n) {
    using std::cbrt;
    yarr[n] = cbrt(xarr[n]);
  }
  const simd<T> y = loada<simd<T> >(yarr);
  return y;
  // constexpr size_t vsize = tuple_size_v<simd<T> >;
  // T xarr[vsize];
  // storeu(xarr, x);
  // T yarr[vsize];
  // for (std::size_t n = 0; n < vsize; ++n) {
  //   using std::cbrt;
  //   yarr[n] = cbrt(xarr[n]);
  // }
  // const simd<T> y = loadu<simd<T> >(yarr);
  // return y;
}

template <typename T>
ARITH_DEVICE ARITH_HOST inline simd<T> sin(const simd<T> &x) {
  constexpr size_t vsize = tuple_size_v<simd<T> >;
  alignas(alignof(simd<T>)) T xarr[vsize];
  storea(xarr, x);
  alignas(alignof(simd<T>)) T yarr[vsize];
  for (std::size_t n = 0; n < x.size(); ++n) {
    using std::sin;
    yarr[n] = sin(xarr[n]);
  }
  const simd<T> y = loada<simd<T> >(yarr);
  return y;
  // constexpr size_t vsize = tuple_size_v<simd<T> >;
  // T xarr[vsize];
  // storeu(xarr, x);
  // T yarr[vsize];
  // for (std::size_t n = 0; n < vsize; ++n) {
  //   using std::sin;
  //   yarr[n] = sin(xarr[n]);
  // }
  // const simd<T> y = loadu<simd<T> >(yarr);
  // return y;
}

////////////////////////////////////////////////////////////////////////////////

// A SIMD vector of booleans, usable with `simd<T>`.
//
// Booleans are special in SIMD operations. There is not just one
// boolean type, but rather one boolean type for each (floating-point
// or integer) type `T`. The reason is that SIMD operations do not
// like changing the number of bits, and hence there are 64-bit
// booleans, 32-bit booleans, 16-bit booleans, etc.
//
// This is described in the NSIMD documentation
// <https://github.com/agenium-scale/nsimd>.

template <typename T> struct simdl {
  typedef T value_type;
#ifndef SIMD_CPU
  nsimd::packl<T> elts;
#else
  bool elts;
#endif

  simdl(const simdl &) = default;
  simdl(simdl &&) = default;
  simdl &operator=(const simdl &) = default;
  simdl &operator=(simdl &&) = default;

  constexpr ARITH_DEVICE ARITH_HOST simdl() {}
  constexpr ARITH_DEVICE ARITH_HOST simdl(bool a) : elts(a) {}
#ifndef SIMD_CPU
  constexpr ARITH_DEVICE ARITH_HOST simdl(const nsimd::packl<T> &elts)
      : elts(elts) {}
#endif

  constexpr ARITH_DEVICE ARITH_HOST std::size_t size() const {
#ifndef SIMD_CPU
    return sizeof(nsimd::packl<T>) / sizeof(T);
#else
    return 1;
#endif
  }

  constexpr ARITH_DEVICE ARITH_HOST bool
  operator[](const std::ptrdiff_t n) const {
    constexpr size_t vsize = tuple_size_v<simdl>;
#ifdef CCTK_DEBUG
    assert(n >= 0 && n < int(vsize));
#endif
    T xarr[vsize];
#ifndef SIMD_CPU
    // TOOD: Introduce `to_mask` for simd/simdl
    storeu(xarr, nsimd::to_mask(elts));
#else
    *xarr = elts;
#endif
    return xarr[n];
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator!(const simdl &x) {
    return !x.elts;
  }
  simdl operator~() const { return !elts; }

  friend ARITH_DEVICE ARITH_HOST simdl operator&&(const simdl &x,
                                                  const simdl &y) {
    return x.elts && y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator&(const simdl &x,
                                                 const simdl &y) {
    return x.elts && y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator||(const simdl &x,
                                                  const simdl &y) {
    return x.elts || y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator|(const simdl &x,
                                                 const simdl &y) {
    return x.elts || y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator^(const simdl &x,
                                                 const simdl &y) {
    return x.elts ^ y.elts;
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator&(const bool a, const simdl &y) {
    return a & y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator|(const bool a, const simdl &y) {
    return a | y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator^(const bool a, const simdl &y) {
    return a ^ y.elts;
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator&(const simdl &x, const bool b) {
    return x.elts & b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator|(const simdl &x, const bool b) {
    return x.elts | b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator^(const simdl &x, const bool b) {
    return x.elts ^ b;
  }

  ARITH_DEVICE ARITH_HOST simdl &operator&=(const simdl &x) {
    return *this = *this & x;
  }
  ARITH_DEVICE ARITH_HOST simdl &operator|=(const simdl &x) {
    return *this = *this | x;
  }
  ARITH_DEVICE ARITH_HOST simdl &operator^=(const simdl &x) {
    return *this = *this ^ x;
  }

  ARITH_DEVICE ARITH_HOST simdl &operator&=(const bool a) {
    return *this = *this & a;
  }
  ARITH_DEVICE ARITH_HOST simdl &operator|=(const bool a) {
    return *this = *this | a;
  }
  ARITH_DEVICE ARITH_HOST simdl &operator^=(const bool a) {
    return *this = *this ^ a;
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator==(const simdl &x,
                                                  const simdl &y) {
    return x.elts == y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator!=(const simdl &x,
                                                  const simdl &y) {
    return x.elts != y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<(const simdl &x,
                                                 const simdl &y) {
    return x.elts < y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>(const simdl &x,
                                                 const simdl &y) {
    return x.elts > y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<=(const simdl &x,
                                                  const simdl &y) {
    return x.elts <= y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>=(const simdl &x,
                                                  const simdl &y) {
    return x.elts >= y.elts;
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator==(const bool a,
                                                  const simdl &y) {
    return a == y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator!=(const bool a,
                                                  const simdl &y) {
    return a != y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<(const bool a, const simdl &y) {
    return a < y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>(const bool a, const simdl &y) {
    return a > y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<=(const bool a,
                                                  const simdl &y) {
    return a <= y.elts;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>=(const bool a,
                                                  const simdl &y) {
    return a >= y.elts;
  }

  friend ARITH_DEVICE ARITH_HOST simdl operator==(const simdl &x,
                                                  const bool b) {
    return x.elts == b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator!=(const simdl &x,
                                                  const bool b) {
    return x.elts != b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<(const simdl &x, const bool b) {
    return x.elts < b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>(const simdl &x, const bool b) {
    return x.elts > b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator<=(const simdl &x,
                                                  const bool b) {
    return x.elts <= b;
  }
  friend ARITH_DEVICE ARITH_HOST simdl operator>=(const simdl &x,
                                                  const bool b) {
    return x.elts >= b;
  }

  friend ARITH_DEVICE ARITH_HOST bool all(const simdl &x) {
#ifndef SIMD_CPU
    return all(x.elts);
#else
    return x.elts;
#endif
  }

  friend ARITH_DEVICE ARITH_HOST simdl andnot(const simdl &x, const simdl &y) {
#ifndef SIMD_CPU
    return andnotl(x.elts, y.elts);
#else
    return x && !y;
#endif
  }

  friend ARITH_DEVICE ARITH_HOST bool any(const simdl &x) {
#ifndef SIMD_CPU
    return any(x.elts);
#else
    return x.elts;
#endif
  }

  friend ARITH_DEVICE ARITH_HOST simd<T>
  if_else(const simdl &cond, const simd<T> &x, const simd<T> &y) {
#ifndef SIMD_CPU
    return if_else1(cond.elts, x.elts, y.elts);
#else
    return cond.elts ? x.elts : y.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST simd<T> if_else(const simdl &cond, const T &a,
                                                 const simd<T> &y) {
    return if_else(cond, simd(a), y);
  }
  friend ARITH_DEVICE ARITH_HOST simd<T> if_else(const simdl &cond,
                                                 const simd<T> &x, const T &b) {
    return if_else(cond, x, simd(b));
  }
  friend ARITH_DEVICE ARITH_HOST simd<T> if_else(const simdl &cond, const T &a,
                                                 const T &b) {
    return if_else(cond, simd(a), simd(b));
  }

  friend ARITH_DEVICE ARITH_HOST simdl<T>
  if_else(const simdl &cond, const simdl<T> &x, const simdl<T> &y) {
#ifndef SIMD_CPU
    return if_else1(cond.elts, x.elts, y.elts);
#else
    return cond.elts ? x.elts : y.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST simdl<T>
  if_else(const simdl &cond, const bool &a, const simdl<T> &y) {
    return if_else(cond, simdl(a), y);
  }
  friend ARITH_DEVICE ARITH_HOST simdl<T>
  if_else(const simdl &cond, const simdl<T> &x, const bool &b) {
    return if_else(cond, x, simdl(b));
  }
  friend ARITH_DEVICE ARITH_HOST simdl<T>
  if_else(const simdl &cond, const bool &a, const bool &b) {
    return if_else(cond, simdl(a), simdl(b));
  }

  friend ARITH_DEVICE ARITH_HOST void storela(T *ptr, const simdl &x) {
#ifndef SIMD_CPU
    storela(ptr, x.elts);
#else
    *ptr = x.elts;
#endif
  }
  friend ARITH_DEVICE ARITH_HOST void storelu(T *ptr, const simdl &x) {
#ifndef SIMD_CPU
    storelu(ptr, x.elts);
#else
    *ptr = x.elts;
#endif
  }

  friend ostream &operator<<(ostream &os, const simdl &x) {
    os << "Ⓑ[";
    constexpr size_t vsize = tuple_size_v<simdl>;
    for (size_t n = 0; n < vsize; ++n) {
      if (n != 0)
        os << ",";
      os << x[n];
    }
    os << "]";
    return os;
  }
};

} // namespace Arith
namespace std {
template <typename T>
struct tuple_size<Arith::simdl<T> >
    : std::integral_constant<std::size_t, sizeof(Arith::simd<T>) / sizeof(T)> {
};
} // namespace std
namespace Arith {

template <typename T> struct zero<simdl<T> > {
  typedef simdl<T> value_type;
  // static constexpr value_type value = simdl<T>(false);
  constexpr ARITH_INLINE operator value_type() const { return simdl<T>(false); }
  constexpr ARITH_INLINE value_type operator()() const {
    return simdl<T>(false);
  }
};

template <typename T> struct one<simdl<T> > {
  typedef simdl<T> value_type;
  // static constexpr value_type value = simdl<T>(true);
  constexpr ARITH_INLINE operator value_type() const { return simdl<T>(true); }
  constexpr ARITH_INLINE value_type operator()() const {
    return simdl<T>(true);
  }
};
} // namespace Arith

#endif // #ifndef SIMD_HXX
