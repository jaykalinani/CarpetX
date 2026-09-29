#include <defs.hxx>
#include <loop_device.hxx>
#include <sum.hxx>
#include <vect.hxx>

#include <cctk.h>
#include <cctk_Arguments.h>
#include <cctk_Parameter.h>
#include <cctk_Parameters.h>

#include <AMReX_GpuAtomic.H>
#include <AMReX_GpuContainers.H>

#include <algorithm>
#include <cmath>
#include <vector>

namespace BHClusterTagging {
using namespace Loop;

constexpr int max_num_extraction_radii = 100;
// These bounds match PunctureTracker's public scalar arrays.
constexpr int max_num_punctures = 100;
constexpr int max_num_puncture_groups = 100;

int carpetx_max_num_levels() {
  int type = 0;
  const void *const value =
      CCTK_ParameterGet("max_num_levels", "CarpetX", &type);
  if (value == nullptr || type != PARAMETER_INT)
    CCTK_ERROR("Could not query CarpetX::max_num_levels");
  return int(*static_cast<const CCTK_INT *>(value));
}

template <typename T>
CCTK_DEVICE CCTK_HOST constexpr T square(const T x) {
  return x * x;
}

extern "C" void BHClusterTagging_EstimateError(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTSX_BHClusterTagging_EstimateError;
  DECLARE_CCTK_PARAMETERS;

  // This criterion, like BoxInBox, is defined only on the Cartesian patch.
  if (cctk_patch != 0)
    return;

  const int current_level = std::ilogb(CCTK_REAL(cctk_levfac[0]));
  const int finest_level = carpetx_max_num_levels() - 1;
  if (current_level >= finest_level) {
    grid.loop_int_device<1, 1, 1>(
        grid.nghostzones,
        [=] CCTK_DEVICE(const PointDesc &p) CCTK_ATTRIBUTE_ALWAYS_INLINE {
          regrid_error(p.I) = 0;
        });
    return;
  }

  const int num_groups = int(pt_num_groups[0]);
  if (num_groups < 0 || num_groups > max_num_puncture_groups)
    CCTK_VERROR("PunctureTracker exported %d groups; BHClusterTagging supports "
                "at most %d",
                num_groups, max_num_puncture_groups);

  const int num_punctures = int(pt_num_tracked[0]);
  if (num_punctures < 0 || num_punctures > max_num_punctures)
    CCTK_VERROR("PunctureTracker exported %d punctures; BHClusterTagging "
                "supports at most %d (the size of "
                "PunctureTracker::pt_group_membership)",
                num_punctures, max_num_punctures);

  // PunctureTracker determines group membership. Reconstruct the group
  // properties from the punctures because GRChombo uses a summed mass and an
  // unweighted arithmetic mean position, whereas PunctureTracker's exported
  // group position is mass weighted.
  auto group_position =
      vect<vect<CCTK_REAL, dim>, max_num_puncture_groups>::make(
          [](const int) { return vect<CCTK_REAL, dim>::pure(0); });
  auto group_mass = vect<CCTK_REAL, max_num_puncture_groups>::pure(0);
  auto group_size = vect<int, max_num_puncture_groups>::pure(0);
  for (int puncture = 0; puncture < num_punctures; ++puncture) {
    const int group = int(pt_group_membership[puncture]);
    if (group < 0 || group >= num_groups)
      CCTK_VERROR("Puncture %d has invalid group membership %d", puncture,
                  group);
    group_position[group] +=
        vect<CCTK_REAL, dim>{pt_loc_x[puncture], pt_loc_y[puncture],
                             pt_loc_z[puncture]};
    group_mass[group] += pt_mass[puncture];
    ++group_size[group];
  }
  for (int group = 0; group < num_groups; ++group) {
    if (group_size[group] <= 0)
      CCTK_VERROR("Puncture group %d has no members", group);
    group_position[group] /= CCTK_REAL(group_size[group]);
  }

  // Print this on every rank, rather than only rank zero, to establish that
  // the local scalar copies seen by the device tagger agree after puncture
  // tracking and its MPI broadcast. A weighted checksum keeps the report
  // compact while still depending on every membership, mass, and coordinate.
  static CCTK_INT last_rank_input_iteration = -1;
  if (verbose_rank_inputs && cctk_iteration != last_rank_input_iteration &&
      cctk_iteration % verbose_every == 0) {
    last_rank_input_iteration = cctk_iteration;
    CCTK_REAL fingerprint = CCTK_REAL(0);
    for (int puncture = 0; puncture < num_punctures; ++puncture) {
      const CCTK_REAL weight = CCTK_REAL(puncture + 1);
      fingerprint +=
          weight * (pt_loc_x[puncture] + CCTK_REAL(3) * pt_loc_y[puncture] +
                    CCTK_REAL(7) * pt_loc_z[puncture] +
                    CCTK_REAL(11) * pt_mass[puncture] +
                    CCTK_REAL(13) * pt_group_membership[puncture]);
    }
    CCTK_VINFO("BHClusterTagging rank input: rank=%d iteration=%d level=%d "
               "component=%d tracked=%d groups=%d fingerprint=%.17g",
               CCTK_MyProc(cctkGH), int(cctk_iteration), current_level,
               int(cctk_component), num_punctures, num_groups,
               double(fingerprint));
  }

  // This host-side report is deliberately emitted only once per Cactus
  // iteration. EstimateError is called on several AMR levels while
  // subcycling, whereas these scalar PunctureTracker inputs are shared by all
  // of them. Keeping one report makes the output useful for diagnosing a
  // regrid without multiplying it by the number of levels.
  static CCTK_INT last_verbose_iteration = -1;
  if (verbose && CCTK_MyProc(cctkGH) == 0 &&
      cctk_iteration != last_verbose_iteration &&
      cctk_iteration % verbose_every == 0) {
    last_verbose_iteration = cctk_iteration;
    CCTK_VINFO("BHClusterTagging input at iteration %d (level %d): "
               "pt_num_tracked=%d, pt_num_groups=%d",
               int(cctk_iteration), current_level, num_punctures, num_groups);
    for (int puncture = 0; puncture < num_punctures; ++puncture) {
      CCTK_VINFO("  puncture %d: group=%d mass=%.17g position=(%.17g,%.17g,%.17g)",
                 puncture, int(pt_group_membership[puncture]),
                 double(pt_mass[puncture]), double(pt_loc_x[puncture]),
                 double(pt_loc_y[puncture]), double(pt_loc_z[puncture]));
    }
    for (int group = 0; group < num_groups; ++group) {
      CCTK_VINFO("  reconstructed group %d: members=%d mass=%.17g "
                 "position=(%.17g,%.17g,%.17g)",
                 group, group_size[group], double(group_mass[group]),
                 double(group_position[group][0]),
                 double(group_position[group][1]),
                 double(group_position[group][2]));
    }
  }
  const auto radii = vect<CCTK_REAL, max_num_extraction_radii>::make(
      [&](const int n) { return extraction_radii[n]; });
  const auto levels = vect<int, max_num_extraction_radii>::make(
      [&](const int n) { return int(extraction_levels[n]); });

  for (int n = 0; n < num_extraction_radii; ++n) {
    if (levels[n] > finest_level)
      CCTK_VERROR("extraction_levels[%d]=%d exceeds CarpetX's finest level %d",
                  n, levels[n], finest_level);
  }
  for (int group = 0; group < num_groups; ++group) {
    if (!(group_mass[group] > 0))
      CCTK_VERROR("Puncture group %d has non-positive mass %.17g", group,
                  double(group_mass[group]));
  }

  const vect<CCTK_REAL, dim> extraction_center{
      extraction_center_x, extraction_center_y, extraction_center_z};
  const CCTK_REAL coarsest_dx = CCTK_REAL(cctk_delta_space[0]);
  const int n_radii = int(num_extraction_radii);
  const int n_min = int(minimum_horizon_points);
  const int n_nested = int(num_nested_levels);
  const int min_level = int(minimum_level_needed);
  const bool do_extraction = bool(activate_extraction);
  const bool inner_grids = bool(use_inner_grids);
  const bool report_coverage =
      verbose_coverage && cctk_iteration % verbose_every == 0;

  if (min_level > finest_level)
    CCTK_VERROR("minimum_level_needed=%d exceeds CarpetX's finest level %d",
                min_level, finest_level);

  auto group_max_level = vect<int, max_num_puncture_groups>::pure(0);
  for (int group = 0; group < num_groups; ++group) {
    const int mass_level =
        int(ceil(log(CCTK_REAL(n_min) * coarsest_dx / group_mass[group]) /
                 log(CCTK_REAL(2))));
    group_max_level[group] =
        std::max(std::min(finest_level, mass_level), min_level);
    if (group_max_level[group] <= 0)
      CCTK_VERROR("Puncture group %d requires non-positive finest level %d; "
                  "increase minimum_level_needed",
                  group, group_max_level[group]);
  }

  // The coverage counters are intentionally local to one AMR component. A
  // schedule callback is not entered by ranks without a component, so an MPI
  // reduction here could deadlock. The rank/level/component fields in the
  // report make these local contributions directly attributable to AMReX
  // boxes. Counter zero is the union of all tags; counters 1..N are the
  // puncture-group contributions before clustering.
  std::vector<unsigned long long> host_coverage;
  amrex::Gpu::DeviceVector<unsigned long long> device_coverage;
  unsigned long long *coverage = nullptr;
  if (report_coverage) {
    host_coverage.assign(size_t(num_groups + 1), 0);
    device_coverage.resize(host_coverage.size());
    amrex::Gpu::copy(amrex::Gpu::hostToDevice, host_coverage.begin(),
                     host_coverage.end(), device_coverage.begin());
    coverage = device_coverage.data();
  }

  grid.loop_int_device<1, 1, 1>(
      grid.nghostzones,
      [=] CCTK_DEVICE(const PointDesc &p) CCTK_ATTRIBUTE_ALWAYS_INLINE {
        bool do_refine = false;

        if (do_extraction) {
          for (int n = 0; n < n_radii; ++n) {
            if (current_level < levels[n]) {
              const auto d = p.X - extraction_center;
              const CCTK_REAL r = sqrt(sum(square(d)));
              // This 20% buffer is part of the GRChombo criterion.
              do_refine |= r < CCTK_REAL(1.2) * radii[n];
            }
          }
        }

        for (int group = 0; group < num_groups; ++group) {
          const CCTK_REAL mass = group_mass[group];
          const int max_level_needed = group_max_level[group];
          const int top_level =
              inner_grids ? finest_level - 1 : max_level_needed - 1;

          if (current_level <= top_level) {
            const int expansion_levels =
                min(max_level_needed - current_level - 1, n_nested);
            const CCTK_REAL expansion =
                pow(CCTK_REAL(2), CCTK_REAL(expansion_levels));
            const auto d = abs(p.X - group_position[group]);
            // GRChombo's get_mod is sqrt(max(dx^2,dy^2,dz^2)), i.e. the
            // infinity norm.  The puncture regions are therefore cubes.
            const CCTK_REAL distance = maximum(d);
            const bool group_refine =
                distance < bh_margin_factor * expansion * mass;
            do_refine |= group_refine;
            if (report_coverage && group_refine)
              amrex::Gpu::Atomic::AddNoRet(&coverage[group + 1], 1ULL);
          }
        }

        regrid_error(p.I) = do_refine ? 1 : 0;
        if (report_coverage && do_refine)
          amrex::Gpu::Atomic::AddNoRet(&coverage[0], 1ULL);
      });

  if (report_coverage) {
    amrex::Gpu::copy(amrex::Gpu::deviceToHost, device_coverage.begin(),
                     device_coverage.end(), host_coverage.begin());
    if (host_coverage[0] > 0) {
      CCTK_VINFO("BHClusterTagging coverage: rank=%d iteration=%d level=%d "
                 "component=%d union_cells=%llu",
                 CCTK_MyProc(cctkGH), int(cctk_iteration), current_level,
                 int(cctk_component), host_coverage[0]);
      for (int group = 0; group < num_groups; ++group)
        if (host_coverage[size_t(group + 1)] > 0)
          CCTK_VINFO("  coverage group=%d cells=%llu", group,
                     host_coverage[size_t(group + 1)]);
    }
  }
}

} // namespace BHClusterTagging

