#ifndef CARPETX_ODESOLVERS_IMEX_TABLEAUX_HXX
#define CARPETX_ODESOLVERS_IMEX_TABLEAUX_HXX

#include <cctk.h>

#include <array>
#include <limits>

namespace ODESolvers {

// Shared by the ordinary and subcycling drivers. The explicit and implicit
// abscissae need not coincide; each partition must use its own stage time.
struct imex_tableau_t {
  int nstages;
  std::array<CCTK_REAL, 4> c_exp, c_imp;
  std::array<std::array<CCTK_REAL, 4>, 4> a_exp, a_imp;
  std::array<CCTK_REAL, 4> b_exp, b_imp;
};

// Izquierdo et al., arXiv:2211.00027v2, Table 10. The implicit part is
// L-stable; this does not imply a uniformly damped mixed stiff limit.
inline constexpr imex_tableau_t imex42l{
    4,
    {0, 1.0 / 2, 1.0 / 2, 1},
    {0, 1.0 / 2, 1.0 / 2, 1},
    {{{0, 0, 0, 0}, {1.0 / 2, 0, 0, 0},
      {0, 1.0 / 2, 0, 0}, {0, 0, 1, 0}}},
    {{{0, 0, 0, 0}, {1.0 / 4, 1.0 / 4, 0, 0},
      {0, 1.0 / 6, 1.0 / 3, 0}, {1.0 / 6, 1.0 / 3, 1.0 / 3, 1.0 / 6}}},
    {1.0 / 6, 1.0 / 3, 1.0 / 3, 1.0 / 6},
    {1.0 / 6, 1.0 / 3, 1.0 / 3, 1.0 / 6}};

// Pareschi and Russo, arXiv:1009.2757, Table 5: IMEX-SSP3(3,3,2).
// gamma=1-1/sqrt(2). This replaces the inconsistent printed CK tableau
// formerly called IMEX32L, not a one-coefficient repair of that tableau.
inline constexpr CCTK_REAL imex_gamma =
    0.2928932188134524755991556378951509607;
inline constexpr imex_tableau_t imex32l{
    3,
    {0, 1, 1.0 / 2, 0},
    {imex_gamma, 1 - imex_gamma, 1.0 / 2, 0},
    {{{0, 0, 0, 0}, {1, 0, 0, 0},
      {1.0 / 4, 1.0 / 4, 0, 0}, {0, 0, 0, 0}}},
    {{{imex_gamma, 0, 0, 0}, {1 - 2 * imex_gamma, imex_gamma, 0, 0},
      {1.0 / 2 - imex_gamma, 0, imex_gamma, 0}, {0, 0, 0, 0}}},
    {1.0 / 6, 1.0 / 6, 2.0 / 3, 0},
    {1.0 / 6, 1.0 / 6, 2.0 / 3, 0}};

// Explicit/implicit midpoint: second order, A-stable but NOT L-stable.
// "Implicit Euler" remains a deprecated compatibility alias of this method.
inline constexpr imex_tableau_t imex122{
    2,
    {0, 1.0 / 2, 0, 0},
    {0, 1.0 / 2, 0, 0},
    {{{0, 0, 0, 0}, {1.0 / 2, 0, 0, 0},
      {0, 0, 0, 0}, {0, 0, 0, 0}}},
    {{{0, 0, 0, 0}, {0, 1.0 / 2, 0, 0},
      {0, 0, 0, 0}, {0, 0, 0, 0}}},
    {0, 1, 0, 0},
    {0, 1, 0, 0}};

constexpr bool imex_tableau_consistent(const imex_tableau_t &tableau) {
  const auto close = [](const CCTK_REAL x, const CCTK_REAL y) {
    constexpr CCTK_REAL tolerance =
        64 * std::numeric_limits<CCTK_REAL>::epsilon();
    const CCTK_REAL difference = x - y;
    return difference >= -tolerance && difference <= tolerance;
  };
  if (tableau.nstages < 1 || tableau.nstages > 4)
    return false;
  CCTK_REAL sum_exp = 0, sum_imp = 0;
  CCTK_REAL ee = 0, ei = 0, ie = 0, ii = 0;
  for (int i = 0; i < tableau.nstages; ++i) {
    CCTK_REAL c_exp = 0, c_imp = 0;
    for (int j = 0; j < tableau.nstages; ++j) {
      if ((j >= i && tableau.a_exp[i][j] != 0) ||
          (j > i && tableau.a_imp[i][j] != 0))
        return false;
      c_exp += tableau.a_exp[i][j];
      c_imp += tableau.a_imp[i][j];
    }
    if (!close(c_exp, tableau.c_exp[i]) ||
        !close(c_imp, tableau.c_imp[i]))
      return false;
    sum_exp += tableau.b_exp[i];
    sum_imp += tableau.b_imp[i];
    ee += tableau.b_exp[i] * c_exp;
    ei += tableau.b_exp[i] * c_imp;
    ie += tableau.b_imp[i] * c_exp;
    ii += tableau.b_imp[i] * c_imp;
  }
  return close(sum_exp, 1) && close(sum_imp, 1) && close(ee, 0.5) &&
         close(ei, 0.5) && close(ie, 0.5) && close(ii, 0.5);
}

static_assert(imex_tableau_consistent(imex42l));
static_assert(imex_tableau_consistent(imex32l));
static_assert(imex_tableau_consistent(imex122));

} // namespace ODESolvers

#endif // CARPETX_ODESOLVERS_IMEX_TABLEAUX_HXX
