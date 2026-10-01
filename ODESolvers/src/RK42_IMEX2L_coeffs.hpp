#ifndef CARPETX_ODESOLVERS_RK42_IMEX2L_COEFFS_HPP
#define CARPETX_ODESOLVERS_RK42_IMEX2L_COEFFS_HPP

#include <cmath>

namespace MultiStepRungeKutta {

// Diagonal of the two-stage stiffly accurate L-stable DIRK used by
// RK42-IMEX2L. gamma = 1 - sqrt(2)/2.
//
//   A = [[gamma, 0], [1 - gamma, gamma]]
//   b = (1 - gamma, gamma)
//   c = (gamma, 1)
//
// Both solves use step gamma*dt. The last row of A equals b, so the
// second solve is the finished step. 2*gamma - gamma^2 = 1/2.

template <typename T> static inline auto rk42_imex2l_gamma() -> T {
  return T(1) - std::sqrt(T(2)) / T(2);
}

} // namespace MultiStepRungeKutta

#endif // CARPETX_ODESOLVERS_RK42_IMEX2L_COEFFS_HPP
