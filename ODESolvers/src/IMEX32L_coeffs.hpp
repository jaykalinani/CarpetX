#ifndef CARPETX_ODESOLVERS_IMEX32L_COEFFS_HPP
#define CARPETX_ODESOLVERS_IMEX32L_COEFFS_HPP

#include <cmath>

namespace ODESolvers {

// Pareschi and Russo, Journal of Scientific Computing 25 (2005) 129-155,
// Table 5 (arXiv:1009.2757). IMEX-SSP3(3,3,2): Shu's SSPRK3 on the explicit
// part and an L-stable DIRK on the implicit part. The diagonal is
// gamma = 1 - 1/sqrt(2), and every implicit solve uses step gamma*dt.
//
// Explicit nodes are (0, 1, 1/2). Implicit nodes are (gamma, 1-gamma, 1/2).
// The two abscissae differ, so the flux and the source of one stage are
// evaluated at different times. Weights are (1/6, 1/6, 2/3).

template <typename T> static inline auto imex32l_gamma() -> T {
  return T(1) - T(1) / std::sqrt(T(2));
}

template <typename T> static inline auto imex32l_b(const int i) -> T {
  if (i == 2)
    return T(2) / T(3);
  return T(1) / T(6);
}

template <typename T> static inline auto imex32l_c_exp(const int i) -> T {
  if (i == 1)
    return T(1);
  if (i == 2)
    return T(1) / T(2);
  return T(0);
}

template <typename T> static inline auto imex32l_c_imp(const int i) -> T {
  const T gamma = imex32l_gamma<T>();
  if (i == 0)
    return gamma;
  if (i == 1)
    return T(1) - gamma;
  return T(1) / T(2);
}

template <typename T> static inline auto imex32l_a_exp(const int i, const int j) -> T {
  if (i == 1 && j == 0)
    return T(1);
  if (i == 2 && (j == 0 || j == 1))
    return T(1) / T(4);
  return T(0);
}

template <typename T> static inline auto imex32l_a_imp(const int i, const int j) -> T {
  const T gamma = imex32l_gamma<T>();
  if (i == 0 && j == 0)
    return gamma;
  if (i == 1 && j == 0)
    return T(1) - T(2) * gamma;
  if (i == 1 && j == 1)
    return gamma;
  if (i == 2 && j == 0)
    return T(1) / T(2) - gamma;
  if (i == 2 && j == 2)
    return gamma;
  return T(0);
}

} // namespace ODESolvers

#endif // CARPETX_ODESOLVERS_IMEX32L_COEFFS_HPP
