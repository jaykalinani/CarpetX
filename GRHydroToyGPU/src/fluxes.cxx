#include <fixmath.hxx>
#include <loop_device.hxx>

#include <cctk.h>
#include <cctk_Arguments_Checked.h>
#include <cctk_Parameters.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace HydroToyGPU {
using namespace std;
using namespace Loop;

enum class reconstruction_t { Godunov, minmod };

namespace {
template <typename T>
inline CCTK_ATTRIBUTE_ALWAYS_INLINE CCTK_DEVICE CCTK_HOST T pow2(T x) {
  return x * x;
}

template <typename T>
inline CCTK_ATTRIBUTE_ALWAYS_INLINE CCTK_DEVICE CCTK_HOST T minmod(const T &x,
                                                                   const T &y) {
  if (signbit(x) != signbit(y))
    return T(0);
  if (fabs(x) < fabs(y))
    return x;
  else
    return y;
}
} // namespace

// Calculate the fluxes in direction `dir`. This function is more
// complex because it has to handle any direction, but as reward,
// there is only one function, not three.
template <int dir> void CalcFlux(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_HydroToyGPU_Fluxes;
  DECLARE_CCTK_PARAMETERS;

  static_assert(dir >= 0 && dir < 3, "");

  // const array<CCTK_REAL, dim> dx = {CCTK_DELTA_SPACE(0), CCTK_DELTA_SPACE(1),
  //                                   CCTK_DELTA_SPACE(2)};
  // const CCTK_REAL dV = dx[0] * dx[1] * dx[2]; // cell volume
  // const CCTK_REAL dA = dV / dx[dir];          // face area

  // Cell centred grid functions
  const GridDescBaseDevice grid(cctkGH);
  constexpr array<int, dim> cell_centred = {1, 1, 1};
  constexpr array<int, dim> vertex_centred = {0, 0, 0};
  const GF3D2layout gf_layout_cell(cctkGH, cell_centred);
  const GF3D2layout gf_layout_vertex(cctkGH, vertex_centred);

  const GF3D2<const CCTK_REAL> gf_gxx(gf_layout_vertex, gxx);
  const GF3D2<const CCTK_REAL> gf_gxy(gf_layout_vertex, gxy);
  const GF3D2<const CCTK_REAL> gf_gxz(gf_layout_vertex, gxz);
  const GF3D2<const CCTK_REAL> gf_gyy(gf_layout_vertex, gyy);
  const GF3D2<const CCTK_REAL> gf_gyz(gf_layout_vertex, gyz);
  const GF3D2<const CCTK_REAL> gf_gzz(gf_layout_vertex, gzz);

  const GF3D2<const CCTK_REAL> gf_alp(gf_layout_vertex, alp);
  const GF3D2<const CCTK_REAL> gf_betax(gf_layout_vertex, betax);
  const GF3D2<const CCTK_REAL> gf_betay(gf_layout_vertex, betay);
  const GF3D2<const CCTK_REAL> gf_betaz(gf_layout_vertex, betaz);

  const GF3D2<const CCTK_REAL> gf_dens(gf_layout_cell, dens);
  const GF3D2<const CCTK_REAL> gf_momx(gf_layout_cell, momx);
  const GF3D2<const CCTK_REAL> gf_momy(gf_layout_cell, momy);
  const GF3D2<const CCTK_REAL> gf_momz(gf_layout_cell, momz);
  const GF3D2<const CCTK_REAL> gf_tau(gf_layout_cell, tau);

  const GF3D2<const CCTK_REAL> gf_rho(gf_layout_cell, rho);
  const GF3D2<const CCTK_REAL> gf_velx(gf_layout_cell, velx);
  const GF3D2<const CCTK_REAL> gf_vely(gf_layout_cell, vely);
  const GF3D2<const CCTK_REAL> gf_velz(gf_layout_cell, velz);
  const GF3D2<const CCTK_REAL> gf_press(gf_layout_cell, press);

  // Face-centred grid functions (in direction `dir`)
  constexpr array<int, dim> face_centred = {!(dir == 0), !(dir == 1),
                                            !(dir == 2)};
  const GF3D2layout gf_fluxlayout(cctkGH, face_centred);

  // Get the grid function pointers for fluxes in direction `dir`
  const array<CCTK_REAL *, dim> fluxdenss = {fxdens, fydens, fzdens};
  const array<CCTK_REAL *, dim> fluxmomxs = {fxmomx, fymomx, fzmomx};
  const array<CCTK_REAL *, dim> fluxmomys = {fxmomy, fymomy, fzmomy};
  const array<CCTK_REAL *, dim> fluxmomzs = {fxmomz, fymomz, fzmomz};
  const array<CCTK_REAL *, dim> fluxtaus = {fxtau, fytau, fztau};
  const GF3D2<CCTK_REAL> gf_fluxdens(gf_fluxlayout, fluxdenss[dir]);
  const GF3D2<CCTK_REAL> gf_fluxmomx(gf_fluxlayout, fluxmomxs[dir]);
  const GF3D2<CCTK_REAL> gf_fluxmomy(gf_fluxlayout, fluxmomys[dir]);
  const GF3D2<CCTK_REAL> gf_fluxmomz(gf_fluxlayout, fluxmomzs[dir]);
  const GF3D2<CCTK_REAL> gf_fluxtau(gf_fluxlayout, fluxtaus[dir]);

  // fdens^i = rho vel^i
  // fmom^i_j = mom_j vel^i + delta^i_j press
  // ftau^i = (tau + press) vel^i

  reconstruction_t reconstruction;
  if (CCTK_EQUALS(reconstruction_method, "Godunov"))
    reconstruction = reconstruction_t::Godunov;
  else if (CCTK_EQUALS(reconstruction_method, "minmod"))
    reconstruction = reconstruction_t::minmod;
  else
    CCTK_ERROR("Unknown value for parameter \"reconstruction_method\"");

  switch (reconstruction) {
  case reconstruction_t::Godunov:
    assert(cctk_nghostzones[dir] >= 1);
  case reconstruction_t::minmod:
    assert(cctk_nghostzones[dir] >= 2);
  }

  constexpr auto DI = PointDesc::DI;
  const auto reconstruct =
      [=] CCTK_DEVICE CCTK_HOST(
          const GF3D2<const CCTK_REAL> &gf_var,
          const PointDesc &p) CCTK_ATTRIBUTE_ALWAYS_INLINE {
        // Neighbouring "plus" and "minus" cell indices
        const auto Imm = p.I - 2 * DI[dir];
        const auto Im = p.I - DI[dir];
        const auto Ip = p.I;
        const auto Ipp = p.I + DI[dir];

        switch (reconstruction) {
        case reconstruction_t::Godunov: {
          CCTK_REAL var_m = gf_var(Im);
          CCTK_REAL var_p = gf_var(Ip);
          return array<CCTK_REAL, 2>{var_m, var_p};
        }
        case reconstruction_t::minmod: {
          CCTK_REAL var_slope_p = gf_var(Ipp) - gf_var(Ip);
          CCTK_REAL var_slope_c = gf_var(Ip) - gf_var(Im);
          CCTK_REAL var_slope_m = gf_var(Im) - gf_var(Imm);
          CCTK_REAL var_m = gf_var(Im) + minmod(var_slope_c, var_slope_m) / 2;
          CCTK_REAL var_p = gf_var(Ip) - minmod(var_slope_p, var_slope_c) / 2;
          return array<CCTK_REAL, 2>{var_m, var_p};
        }
        default:
          CCTK_BUILTIN_UNREACHABLE();
        }
      };

  const auto calcflux = [=] CCTK_DEVICE CCTK_HOST(
                            CCTK_REAL var_m, CCTK_REAL var_p, CCTK_REAL flux_m,
                            CCTK_REAL flux_p) CCTK_ATTRIBUTE_ALWAYS_INLINE {
    CCTK_REAL lambda_m = +1.0;
    CCTK_REAL lambda_p = -1.0;
    CCTK_REAL llf =
        0.5 * ((flux_m + flux_p) -
               fmax(fabs(lambda_m), fabs(lambda_p)) * (var_p - var_m));
    // return dA * llf;
    return llf;
  };

  grid.loop_int_device<face_centred[0], face_centred[1], face_centred[2]>(
      grid.nghostzones, [=] CCTK_DEVICE CCTK_HOST(
                            const PointDesc &p) CCTK_ATTRIBUTE_ALWAYS_INLINE {
        // Reconstruct values from the cells on left and right side of this face
        array<CCTK_REAL, 2> rho_r = reconstruct(gf_rho, p);
        array<CCTK_REAL, 2> velx_r = reconstruct(gf_velx, p);
        array<CCTK_REAL, 2> vely_r = reconstruct(gf_vely, p);
        array<CCTK_REAL, 2> velz_r = reconstruct(gf_velz, p);
        array<CCTK_REAL, 2> press_r = reconstruct(gf_press, p);
        array<array<CCTK_REAL, 2>, 3> vels_r = {velx_r, vely_r, velz_r};
        array<CCTK_REAL, 2> vel_r = vels_r[dir];

        array<CCTK_REAL, 2> tau_r;
        for (int f = 0; f < 2; ++f) {
          CCTK_REAL ekin =
              0.5 * rho_r[f] *
              (pow2(velx_r[f]) + pow2(vely_r[f]) + pow2(velz_r[f]));
          CCTK_REAL eint = press_r[f] / (gamma - 1);
          tau_r[f] = ekin + eint;
        }

        CCTK_REAL gxx_r = 0;
        // loop over cube corners except for direction in whihc we reconstruct
        for(int dk = 0 ; dk < (dir == 2 ? 1 : 2) ; ++dk) {
        for(int dj = 0 ; dj < (dir == 1 ? 1 : 2) ; ++dj) {
        for(int di = 0 ; di < (dir == 0 ? 1 : 2) ; ++di) {
          gxx_r += gf_gxx(p.I + p.DI[0]*di + p.DI[1]*dj + p.DI[2]*dk);
        }}}
        gxx_r /= 4;
        // TODO: do the same for other metric variables
        CCTK_REAL sqrt_detg_r = sqrt(gxx_r...);

        array<CCTK_REAL, 2> dens_r = {
                sqrt_detg_r * rho_r[0] * w_lorentz_r[0],
                sqrt_detg_r * rho_r[1] * w_lorentz_r[1]
        };
        const CCTK_REAL flux_dens_m = dens_r[0] * (vel_r[0] - beta_r[0] / alp_r[0]);
        const CCTK_REAL flux_dens_p = dens_r[1] * (vel_r[1] - beta_r[1] / alp_r[1]);

        gf_fluxdens(p.I) = calcflux(dens_r[0], dens_r[1],
                                    flux_dens_m, flux_dens_p);
       
        gf_fluxmomx(p.I) =
            calcflux(rho_r[0] * velx_r[0], rho_r[1] * velx_r[1],
                     rho_r[0] * velx_r[0] * vel_r[0] + (dir == 0) * press_r[0],
                     rho_r[1] * velx_r[1] * vel_r[1] + (dir == 0) * press_r[1]);
        gf_fluxmomy(p.I) =
            calcflux(rho_r[0] * vely_r[0], rho_r[1] * vely_r[1],
                     rho_r[0] * vely_r[0] * vel_r[0] + (dir == 1) * press_r[0],
                     rho_r[1] * vely_r[1] * vel_r[1] + (dir == 1) * press_r[1]);
        gf_fluxmomz(p.I) =
            calcflux(rho_r[0] * velz_r[0], rho_r[1] * velz_r[1],
                     rho_r[0] * velz_r[0] * vel_r[0] + (dir == 2) * press_r[0],
                     rho_r[1] * velz_r[1] * vel_r[1] + (dir == 2) * press_r[1]);
        gf_fluxtau(p.I) =
            calcflux(tau_r[0], tau_r[1], (tau_r[0] + press_r[0]) * vel_r[0],
                     (tau_r[1] + press_r[1]) * vel_r[1]);
      });
}

extern "C" void HydroToyGPU_Fluxes(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_HydroToyGPU_Fluxes;
  DECLARE_CCTK_PARAMETERS;

  CalcFlux<0>(cctkGH);
  CalcFlux<1>(cctkGH);
  CalcFlux<2>(cctkGH);
}

} // namespace HydroToyGPU
