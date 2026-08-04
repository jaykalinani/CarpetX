#include "solve.hxx"
#include <subcycling.hxx>

// For FillPatch_ProlongateToBand (band->band prolongation of the RK k-stages).
// TODO: Don't include files from other thorns; create a proper interface.
#include "../../CarpetX/src/fillpatch.hxx"

#include <AMReX_MultiFab.H>

namespace ODESolvers {

constexpr int max_num_rk_stages = 4;

namespace {

struct solve_setup_t {
  statecomp_t var, rhs;
  std::vector<int> var_groups, rhs_groups, dep_groups;
  int nvars = 0;
};

// Collect evolved groups into statecomp_t bundles. The old-state anchor is now
// a scratch copy of var(tl=0) made by the solver (no extra timelevel); the RK
// k-stages live as coarse-fine bands on each group's GroupData. Operates on
// CarpetX::active_levels; no cGH needed.
solve_setup_t collect_solve_setup() {
  solve_setup_t s;
  s.var.timelevel = 0;
  s.rhs.timelevel = 0;
  bool do_accumulate_nvars = true;
  assert(CarpetX::active_levels);
  CarpetX::active_levels->loop_serially([&](const auto &leveldata) {
    for (const auto &groupdataptr : leveldata.groupdata) {
      // TODO: add support for evolving grid scalars
      if (groupdataptr == nullptr)
        continue;

      auto &groupdata = *groupdataptr;
      const int rhs_gi = get_group_rhs(groupdata.groupindex);
      if (rhs_gi >= 0) {
        assert(rhs_gi != groupdata.groupindex);
        auto &rhs_groupdata = *leveldata.groupdata.at(rhs_gi);
        assert(rhs_groupdata.numvars == groupdata.numvars);
        s.var.groupdatas.push_back(&groupdata);
        s.var.mfabs.push_back(groupdata.mfab.at(0).get());
        s.rhs.groupdatas.push_back(&rhs_groupdata);
        s.rhs.mfabs.push_back(rhs_groupdata.mfab.at(0).get());

        if (do_accumulate_nvars) {
          s.nvars += groupdata.numvars;
          s.var_groups.push_back(groupdata.groupindex);
          s.rhs_groups.push_back(rhs_gi);
          const auto &dependents = get_group_dependents(groupdata.groupindex);
          s.dep_groups.insert(s.dep_groups.end(), dependents.begin(),
                              dependents.end());
        }
      }
    }
    do_accumulate_nvars = false;
  });

  {
    std::sort(s.var_groups.begin(), s.var_groups.end());
    const auto last = std::unique(s.var_groups.begin(), s.var_groups.end());
    assert(last == s.var_groups.end());
  }

  {
    std::sort(s.rhs_groups.begin(), s.rhs_groups.end());
    const auto last = std::unique(s.rhs_groups.begin(), s.rhs_groups.end());
    assert(last == s.rhs_groups.end());
  }

  // Add RHS variables to dependent variables
  s.dep_groups.insert(s.dep_groups.end(), s.rhs_groups.begin(),
                      s.rhs_groups.end());

  {
    std::sort(s.dep_groups.begin(), s.dep_groups.end());
    const auto last = std::unique(s.dep_groups.begin(), s.dep_groups.end());
    s.dep_groups.erase(last, s.dep_groups.end());
  }

  for (const int gi : s.var_groups)
    assert(std::find(s.dep_groups.begin(), s.dep_groups.end(), gi) ==
           s.dep_groups.end());
  for (const int gi : s.rhs_groups)
    assert(std::find(s.var_groups.begin(), s.var_groups.end(), gi) ==
           s.var_groups.end());

  return s;
}

} // namespace

extern "C" void ODESolvers_Solve_Subcycling(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_ODESolvers_Solve_Subcycling;
  DECLARE_CCTK_PARAMETERS;

  static bool did_output = false;
  if (verbose || !did_output)
    CCTK_VINFO("Integrator is %s", method);
  did_output = true;

  static Timer timer("ODESolvers::Solve");
  Interval interval(timer);

  const CCTK_REAL dt = CCTK_DELTA_TIME;
  const CCTK_REAL saved_delta_time = cctkGH->cctk_delta_time;
  current_step_delta_time = dt;

  static Timer timer_setup("ODESolvers::Solve::setup");
  std::optional<Interval> interval_setup(timer_setup);

  auto setup = collect_solve_setup();
  auto &var = setup.var;
  auto &rhs = setup.rhs;
  auto &var_groups = setup.var_groups;
  auto &rhs_groups = setup.rhs_groups;
  auto &dep_groups = setup.dep_groups;
  const int nvars = setup.nvars;
  if (verbose)
    CCTK_VINFO("  Integrating %d variables", nvars);
  if (nvars == 0)
    CCTK_VWARN(CCTK_WARN_ALERT, "Integrating %d variables", nvars);

  interval_setup.reset();

  {
    static Timer timer_alloc_temps("ODESolvers::Solve::alloc_temps");
    Interval interval_alloc_temps(timer_alloc_temps);
    statecomp_t::init_tmp_mfabs();
  }

  // Allocate the coarse-fine RK k-stage bands single-threaded. The source-band
  // geometry reads the next-finer level, so this must run once all levels
  // exist; like build_cf_mask it opens its own MFIter/OpenMP region and so must
  // not run inside a parallel consume. This is the band allocation site that
  // setks, the band->band prolongation, and the dense-output kernel rely on.
  active_levels->loop_serially([&](const auto &leveldata) {
    for (const int gi : var_groups) {
      const auto &gd = *leveldata.groupdata.at(gi);
      leveldata.build_cf_mask(gd.indextype, gd.nghostzones);
      leveldata.build_bands(gd);
    }
  });

  const CCTK_REAL saved_time = cctkGH->cctk_time;
  const CCTK_REAL old_time = cctkGH->cctk_time - dt;

  static Timer timer_lincomb("ODESolvers::Solve::lincomb");
  static Timer timer_rhs("ODESolvers::Solve::rhs");
  static Timer timer_poststep("ODESolvers::Solve::poststep");

  const auto calcrhs = [&](const int n) {
    Interval interval_rhs(timer_rhs);
    if (verbose)
      CCTK_VINFO("Calculating RHS #%d at t=%g", n, double(cctkGH->cctk_time));
    CallScheduleGroup(cctkGH, "ODESolvers_RHS");
    rhs.check_valid(make_valid_int(),
                    "ODESolvers after calling ODESolvers_RHS");
  };
  const auto calcimplicitrhs = [&](const int n) {
    Interval interval_rhs(timer_rhs);
    rhs.set_zero(make_valid_int());
    if (verbose)
      CCTK_VINFO("Calculating implicit RHS #%d at t=%g", n,
                 double(cctkGH->cctk_time));
    CallScheduleGroup(cctkGH, "ODESolvers_ImplicitRHS");
    rhs.check_valid(make_valid_int(),
                    "ODESolvers after calling ODESolvers_ImplicitRHS");
  };
  const auto calcimplicitstep = [&](const int n, const CCTK_REAL step_dt) {
    if (step_dt == 0)
      return;
    var.check_valid(make_valid_all(),
                    "ODESolvers before calling ODESolvers_ImplicitStep");
    if (verbose)
      CCTK_VINFO("Taking implicit step #%d at t=%g with dt=%g", n,
                 double(cctkGH->cctk_time), double(step_dt));
    *const_cast<CCTK_REAL *>(&cctkGH->cctk_delta_time) =
        step_dt * cctkGH->cctk_timefac;
    CallScheduleGroup(cctkGH, "ODESolvers_ImplicitStep");
    *const_cast<CCTK_REAL *>(&cctkGH->cctk_delta_time) = saved_delta_time;
    var.check_valid(make_valid_int(),
                    "ODESolvers after calling ODESolvers_ImplicitStep");
    mark_invalid(dep_groups);
  };
  // t = t_0 + c
  // var = a_0 * var + \Sum_i a_i * var_i
  const auto calcupdate = [&](const int n, const CCTK_REAL c,
                              const CCTK_REAL a0, const auto &as,
                              const auto &vars) {
    {
      Interval interval_lincomb(timer_lincomb);
      if (verbose)
        CCTK_VINFO("Calculated new state #%d at t=%g", n,
                   double(cctkGH->cctk_time));
      statecomp_t::lincomb(var, a0, as, vars, make_valid_int());
      var.check_valid(make_valid_int(),
                      "ODESolvers after defining new state vector");
      mark_invalid(dep_groups);
    }
    {
      Interval interval_poststep(timer_poststep);
      *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) = old_time + c;
    }
  };
  // calling ODESolvers_PostStep Group
  const auto calcpoststep = [&]() {
    CallScheduleGroup(cctkGH, "ODESolvers_PostStep");
  };
  const auto calcpreimplicit = [&]() {
    assert(!in_pre_implicit_stage);
    in_pre_implicit_stage = 1;
    CallScheduleGroup(cctkGH, "ODESolvers_PostStep");
    in_pre_implicit_stage = 0;
  };
  // Calculate Ys on the mesh refinement boundary. Explicit RK methods use
  // dense output from their k-stage bands; IMEX methods interpolate between
  // accepted coarse-step endpoints.
  const auto calcys_rmbnd = [&](const int stage,
                                const CCTK_REAL stage_time = -1.0) {
    if (verbose)
      CCTK_VINFO(
          "Fill refinement boundary ghost zones using Ys for stage #%d at t=%g",
          stage, double(cctkGH->cctk_time));

    active_levels->loop_coarse_to_fine([&](auto &leveldata) {
      const int level = leveldata.level;
      if (level == 0)
        return;

      const auto &patchdata = ghext->patchdata.at(leveldata.patch);
      const auto &prev_leveldata = patchdata.leveldata.at(level - 1);
      // Virtual end-of-step stage = num_rk_stages + 1 (RK4 -> 5, SSPRK3 -> 4).
      const int virtual_end = ghext->num_rk_stages + 1;
      CCTK_REAL xsi =
          (leveldata.iteration == prev_leveldata.iteration) ? 0.5 : 0.0;
      if (stage_time >= 0.0) {
        xsi += 0.5 * stage_time;
        Subcycling::CalcYfFromEndpoints_MFlevel(leveldata, var_groups,
                                                /*Yf_tl=*/0, xsi);
      } else {
        if (stage == virtual_end)
          xsi += 0.5;
        const int stage0 = (stage == virtual_end ? 1 : stage);
        if (ghext->num_rk_stages == 3)
          Subcycling::CalcYfFromKcs_MFlevel<3>(
              leveldata, var_groups, /*Yf_tl=*/0, dt * 2, xsi, stage0);
        else
          Subcycling::CalcYfFromKcs_MFlevel<4>(
              leveldata, var_groups, /*Yf_tl=*/0, dt * 2, xsi, stage0);
      }
    });
    synchronize();

    // calcys_rmbnd writes only coarse-fine ghosts. The stage lincomb also
    // touches the allocated halo even though only its interior is valid, so
    // refresh same-level and physical-boundary ghosts before any post-step or
    // RHS routine can consume them. GhostOnly preserves the temporally
    // interpolated coarse-fine values written above.
    SyncGroupsByDirIGhostOnly(cctkGH, var_groups.size(), var_groups.data(),
                              nullptr, /*tl=*/0);
    synchronize();
    var.set_valid(make_valid_all());
    var.check_valid(make_valid_all(),
                    "ODESolvers after filling stage ghost zones");
  };
  // set ks in the interior which will be used for prolongation later
  const auto setks = [&](const int stage) {
    if (verbose)
      CCTK_VINFO(
          "Set interior Ks for stage #%d at t=%g, to be prolongated later",
          stage, double(cctkGH->cctk_time));
    const int s = stage - 1;
    active_levels->loop_coarse_to_fine([&](const auto &restrict leveldata) {
      // rhs_groups[i] and var_groups[i] are paired by sort order.
      for (size_t i = 0; i < rhs_groups.size(); ++i) {
        const auto &rhs_groupdata = *leveldata.groupdata.at(rhs_groups[i]);
        // The k-stage bands live on the evolved group's GroupData.
        const auto &groupdata = *leveldata.groupdata.at(var_groups[i]);
        // The finest level has no source band (no children to prolongate to).
        if (!groupdata.ks_source_band[s])
          continue;
        auto &rhs_mf = *rhs_groupdata.mfab.at(0);
        auto &src_band = *groupdata.ks_source_band[s];
        assert(src_band.ixType() == rhs_mf.ixType());
        assert(src_band.nComp() == rhs_mf.nComp());
        const auto &geom = CarpetX::ghext->patchdata.at(leveldata.patch)
                               .amrcore->Geom(leveldata.level);
        // Fill the source band from the RHS interior, including periodic
        // images when a refinement region overlaps a periodic boundary. The
        // source-band boxes can extend outside the domain even though they
        // have no ghost cells of their own.
        src_band.ParallelCopy(rhs_mf, 0, 0, src_band.nComp(), amrex::IntVect{0},
                              amrex::IntVect{0}, geom.periodicity());
      }
    });
    synchronize();
  };
  // Capture u(t_n) = var(tl=0) into each level's old_source_band (interior
  // only), for prolongation into the children's old_consumer_band. Like setks
  // but from var(tl=0) once per step. The finest level has no source band.
  const auto fill_old_source_band = [&]() {
    active_levels->loop_coarse_to_fine([&](const auto &restrict leveldata) {
      for (const int gi : var_groups) {
        const auto &groupdata = *leveldata.groupdata.at(gi);
        if (!groupdata.old_source_band)
          continue;
        auto &var_mf = *groupdata.mfab.at(0);
        auto &src_band = *groupdata.old_source_band;
        assert(src_band.ixType() == var_mf.ixType());
        assert(src_band.nComp() == var_mf.nComp());
        const auto &geom = CarpetX::ghext->patchdata.at(leveldata.patch)
                               .amrcore->Geom(leveldata.level);
        src_band.ParallelCopy(var_mf, 0, 0, src_band.nComp(), amrex::IntVect{0},
                              amrex::IntVect{0}, geom.periodicity());
      }
    });
    synchronize();
  };
  // Prolongate the coarse-fine bands from the parent (band->band), filling each
  // fine level's consumer bands: the RK k-stage bands (written by setks) and
  // the single old-state band (written by fill_old_source_band). Both feed
  // calcys_rmbnd.
  const auto prolongate_bands = [&](const bool imex_endpoints_only = false) {
    active_levels->loop_coarse_to_fine([&](const auto &restrict leveldata) {
      const int level = leveldata.level;
      if (level == 0)
        return;
      const auto &patchdata = ghext->patchdata.at(leveldata.patch);
      const auto &coarseleveldata = patchdata.leveldata.at(level - 1);
      // Prolongate the bands are redundant for the second fine iteration.
      if (leveldata.iteration == coarseleveldata.iteration)
        return;
      const auto &fgeom = patchdata.amrcore->Geom(level);
      const auto &cgeom = patchdata.amrcore->Geom(level - 1);
      for (size_t i = 0; i < var_groups.size(); ++i) {
        const auto &groupdata = *leveldata.groupdata.at(var_groups[i]);
        const auto &coarsegroupdata =
            *coarseleveldata.groupdata.at(var_groups[i]);
        amrex::Interpolater *const interpolator = groupdata.interpolator;
        const int num_bands = imex_endpoints_only ? 1 : max_num_rk_stages;
        for (int s = 0; s < num_bands; ++s) {
          // IMEX slot zero is filled directly from the parent's accepted full
          // state. Explicit RK slots use the compact k-stage source bands.
          const amrex::MultiFab *const source =
              imex_endpoints_only
                  ? coarsegroupdata.mfab.at(0).get()
                  : coarsegroupdata.ks_source_band[s].get();
          if (!groupdata.ks_consumer_band[s] || !source)
            continue;
          CarpetX::FillPatch_ProlongateToBand(
              groupdata, coarsegroupdata, *groupdata.ks_consumer_band[s],
              *source, fgeom, cgeom, interpolator, groupdata.bcrecs);
        }
        // Old-state band (single), reusing the same band->band helper.
        if (groupdata.old_consumer_band && coarsegroupdata.old_source_band)
          CarpetX::FillPatch_ProlongateToBand(
              groupdata, coarsegroupdata, *groupdata.old_consumer_band,
              *coarsegroupdata.old_source_band, fgeom, cgeom, interpolator,
              groupdata.bcrecs);
      }
    });
    synchronize();
  };
  const auto run_imex =
      [&](const vector<CCTK_REAL> &cs, const vector<vector<CCTK_REAL> > &a_exp,
          const vector<vector<CCTK_REAL> > &a_imp,
          const vector<CCTK_REAL> &b_exp, const vector<CCTK_REAL> &b_imp) {
        const int nstages = cs.size();
        assert(int(a_exp.size()) == nstages);
        assert(int(a_imp.size()) == nstages);
        assert(int(b_exp.size()) == nstages);
        assert(int(b_imp.size()) == nstages);

        const auto old = var.copy(make_valid_all());
        vector<statecomp_t> fks;
        vector<statecomp_t> gks;
        fks.reserve(nstages);
        gks.reserve(nstages);

        if (var_groups.size() > 0) {
          fill_old_source_band();
          prolongate_bands(/*imex_endpoints_only=*/true);
        }

        const auto define_stage_base = [&](const int stage) {
          vector<CCTK_REAL> factors;
          vector<const statecomp_t *> srcs;
          factors.reserve(1 + 2 * stage);
          srcs.reserve(1 + 2 * stage);
          factors.push_back(1.0);
          srcs.push_back(&old);
          for (int j = 0; j < stage; ++j) {
            if (a_exp.at(stage).at(j) != 0) {
              factors.push_back(dt * a_exp.at(stage).at(j));
              srcs.push_back(&fks.at(j));
            }
            if (a_imp.at(stage).at(j) != 0) {
              factors.push_back(dt * a_imp.at(stage).at(j));
              srcs.push_back(&gks.at(j));
            }
          }
          statecomp_t::lincomb(var, 0.0, factors, srcs, make_valid_int());
          var.check_valid(make_valid_int(),
                          "ODESolvers after defining IMEX stage base");
          mark_invalid(dep_groups);
          *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) =
              old_time + cs.at(stage) * dt;
        };

        for (int stage = 0; stage < nstages; ++stage) {
          if (stage > 0) {
            define_stage_base(stage);
            calcys_rmbnd(stage + 1, cs.at(stage));
            calcpreimplicit();
            // PostStep routines may update evolved interiors (for example,
            // conservative-to-primitive atmosphere repair) and their regular
            // subcycling SYNC does not refill coarse-fine ghosts during a fine
            // substep. Restore the time-interpolated refinement boundary
            // before the diagonal implicit solve consumes the stage state.
            calcys_rmbnd(stage + 1, cs.at(stage));
          }

          const CCTK_REAL diag = a_imp.at(stage).at(stage);
          statecomp_t g;
          if (diag == 0) {
            calcimplicitrhs(stage + 1);
            CallScheduleGroup(cctkGH, "ODESolvers_AfterImplicitRHS");
            g = rhs.copy(make_valid_int());
          } else {
            // Scratch copies share the evolved group's validity metadata.
            // Preserve the stage ghost validity while retaining the
            // pre-implicit interior used to recover g(Y_i).
            const auto base = var.copy(make_valid_all());
            calcimplicitstep(stage + 1, diag * dt);
            calcys_rmbnd(stage + 1, cs.at(stage));
            calcpoststep();

            // Use the update accepted by the diagonal solver as the effective
            // implicit operator. This remains consistent when that solver
            // applies a limiter, floor, or realizability repair.
            statecomp_t::lincomb(
                rhs, 0.0,
                vector<CCTK_REAL>{1.0 / (diag * dt), -1.0 / (diag * dt)},
                vector<const statecomp_t *>{&var, &base}, make_valid_int());
            rhs.check_valid(make_valid_int(),
                            "ODESolvers effective implicit RHS");
            CallScheduleGroup(cctkGH, "ODESolvers_AfterImplicitRHS");
            g = rhs.copy(make_valid_int());
          }

          calcrhs(stage + 1);
          auto f = rhs.copy(make_valid_int());
          fks.push_back(std::move(f));
          gks.push_back(std::move(g));
        }

        vector<CCTK_REAL> factors;
        vector<const statecomp_t *> srcs;
        factors.reserve(1 + 2 * nstages);
        srcs.reserve(1 + 2 * nstages);
        factors.push_back(1.0);
        srcs.push_back(&old);
        for (int stage = 0; stage < nstages; ++stage) {
          if (b_exp.at(stage) != 0) {
            factors.push_back(dt * b_exp.at(stage));
            srcs.push_back(&fks.at(stage));
          }
          if (b_imp.at(stage) != 0) {
            factors.push_back(dt * b_imp.at(stage));
            srcs.push_back(&gks.at(stage));
          }
        }
        statecomp_t::lincomb(var, 0.0, factors, srcs, make_valid_int());
        var.check_valid(make_valid_int(),
                        "ODESolvers after defining IMEX final state");
        mark_invalid(dep_groups);
        *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) = old_time + dt;
        calcys_rmbnd(nstages + 1, 1.0);
        calcpoststep();
      };

  *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) = old_time;

  if (CCTK_EQUALS(method, "constant")) {

    // y1 = y0

    // do nothing

  } else if (CCTK_EQUALS(method, "RK4")) {

    // k1 = f(y0)
    // k2 = f(y0 + h/2 k1)
    // k3 = f(y0 + h/2 k2)
    // k4 = f(y0 + h k3)
    // y1 = y0 + h/6 k1 + h/3 k2 + h/3 k3 + h/6 k4

    // Scratch copy of u(t_n) = var(tl=0), the RK4 interior anchor y0. At one
    // timelevel var(tl=0) holds the previous step's result, so no init copy is
    // needed (mirrors the non-subcycling solver).
    const auto old = var.copy(make_valid_all());

    // Capture u(t_n) into the old bands before the RK stages overwrite var,
    // then prolongate the parent's bands (old + k-stage) into this level's
    // consumer bands.
    if (var_groups.size() > 0) {
      fill_old_source_band();
      prolongate_bands();
    }

    // k1 = f(Y1)
    calcrhs(1);
    setks(1); // interior only
    const auto kaccum = rhs.copy(make_valid_int());
    calcupdate(1, dt / 2, 1.0, reals<1>{dt / 2}, states<1>{&rhs});
    calcys_rmbnd(2); // refinement boundary only
    calcpoststep();

    // k2 = f(Y2)
    calcrhs(2);
    setks(2); // interior only
    statecomp_t::lincomb(kaccum, 1.0, reals<1>{2.0}, states<1>{&rhs},
                         make_valid_int());
    calcupdate(2, dt / 2, 0.0, reals<2>{1.0, dt / 2}, states<2>{&old, &rhs});
    calcys_rmbnd(3); // refinement boundary only
    calcpoststep();

    // k3 = f(Y3)
    calcrhs(3);
    setks(3); // interior only
    statecomp_t::lincomb(kaccum, 1.0, reals<1>{2.0}, states<1>{&rhs},
                         make_valid_int());
    calcupdate(3, dt, 0.0, reals<2>{1.0, dt}, states<2>{&old, &rhs});
    calcys_rmbnd(4); // refinement boundary only
    calcpoststep();

    // k4 = f(Y4)
    calcrhs(4);
    setks(4); // interior only
    calcupdate(4, dt, 0.0, reals<3>{1.0, dt / 6, dt / 6},
               states<3>{&old, &kaccum, &rhs});
    calcys_rmbnd(5); // refinement boundary only
    calcpoststep();

    // No calcys_rmbnd(1) here: refinement-boundary ghosts are kept aligned by
    // subcycling-aware POSTRESTRICT SYNCs. The post-recovery case is handled
    // by ODESolvers_Solve_Subcycling_Recovery at CCTK_CPINITIAL.

  } else if (CCTK_EQUALS(method, "SSPRK3")) {

    // k1 = f(y0)
    // k2 = f(y0 + h k1)
    // k3 = f(y0 + h/4 k1 + h/4 k2)
    // y1 = y0 + h/6 k1 + h/6 k2 + 2/3 h k3

    assert(ghext->num_rk_stages == 3);

    // Scratch copy of u(t_n) = var(tl=0), the SSPRK3 interior anchor y0.
    const auto old = var.copy(make_valid_all());

    // Capture u(t_n) into the old bands before the RK stages overwrite var,
    // then prolongate the parent's bands (old + k-stage) into this level's
    // consumer bands.
    if (var_groups.size() > 0) {
      fill_old_source_band();
      prolongate_bands();
    }

    // k1 = f(Y1)
    calcrhs(1);
    setks(1); // interior only
    const auto k1 = rhs.copy(make_valid_int());
    calcupdate(1, dt, 1.0, reals<1>{dt}, states<1>{&rhs}); // var = y0 + dt*k1
    calcys_rmbnd(2); // refinement boundary only
    calcpoststep();

    // k2 = f(Y2)
    calcrhs(2);
    setks(2); // interior only
    const auto k2 = rhs.copy(make_valid_int());
    calcupdate(2, dt / 2, 0.0, reals<3>{1.0, dt / 4, dt / 4},
               states<3>{&old, &k1, &k2});
    calcys_rmbnd(3); // refinement boundary only
    calcpoststep();

    // k3 = f(Y3)
    calcrhs(3);
    setks(3); // interior only
    calcupdate(3, dt, 0.0, reals<4>{1.0, dt / 6, dt / 6, 2 * dt / 3},
               states<4>{&old, &k1, &k2, &rhs});
    calcys_rmbnd(4); // virtual end-of-step (num_rk_stages + 1)
    calcpoststep();

  } else if (CCTK_EQUALS(method, "IMEX42L")) {

    const vector<CCTK_REAL> c{0, CCTK_REAL(1) / 2, CCTK_REAL(1) / 2, 1};
    const vector<vector<CCTK_REAL> > a_exp{
        {0, 0, 0, 0},
        {CCTK_REAL(1) / 2, 0, 0, 0},
        {0, CCTK_REAL(1) / 2, 0, 0},
        {0, 0, 1, 0},
    };
    const vector<vector<CCTK_REAL> > a_imp{
        {0, 0, 0, 0},
        {CCTK_REAL(1) / 4, CCTK_REAL(1) / 4, 0, 0},
        {0, CCTK_REAL(1) / 6, CCTK_REAL(1) / 3, 0},
        {CCTK_REAL(1) / 6, CCTK_REAL(1) / 3, CCTK_REAL(1) / 3,
         CCTK_REAL(1) / 6},
    };
    const vector<CCTK_REAL> b{CCTK_REAL(1) / 6, CCTK_REAL(1) / 3,
                             CCTK_REAL(1) / 3, CCTK_REAL(1) / 6};
    run_imex(c, a_exp, a_imp, b, b);

  } else if (CCTK_EQUALS(method, "IMEX32L")) {

    assert(ghext->num_rk_stages == 3);
    const vector<CCTK_REAL> c{0, 1, CCTK_REAL(1) / 2};
    const vector<vector<CCTK_REAL> > a_exp{
        {0, 0, 0},
        {1, 0, 0},
        {CCTK_REAL(1) / 4, CCTK_REAL(1) / 4, 0},
    };
    const vector<vector<CCTK_REAL> > a_imp{
        {0, 0, 0},
        {CCTK_REAL(1) / 2, CCTK_REAL(1) / 2, 0},
        {CCTK_REAL(1) / 6, CCTK_REAL(1) / 6, CCTK_REAL(2) / 3},
    };
    const vector<CCTK_REAL> b{CCTK_REAL(1) / 6, CCTK_REAL(1) / 6,
                             CCTK_REAL(2) / 3};
    run_imex(c, a_exp, a_imp, b, b);

  } else {
    assert(0);
  }

  {
    static Timer timer_free_temps("ODESolvers::Solve::free_temps");
    Interval interval_free_temps(timer_free_temps);
    statecomp_t::free_tmp_mfabs();
  }

  // Reset current time
  *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) = saved_time;

  // TODO: Update time here, and not during time level cycling in the driver
}

extern "C" void ODESolvers_Solve_Subcycling_Recovery(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_ODESolvers_Solve_Subcycling_Recovery;
  DECLARE_CCTK_PARAMETERS;

  // Skip on fresh initialization; cctk_iteration > 0 only on recovery.
  if (cctk_iteration <= 0)
    return;

  if (verbose)
    CCTK_VINFO("Subcycling recovery: refilling refinement-boundary ghosts "
               "(spatial prolongation on time-aligned levels, dense output "
               "from restored consumer bands otherwise)");

  static Timer timer("ODESolvers::Solve_Subcycling_Recovery");
  Interval interval(timer);

  const CCTK_REAL dt = CCTK_DELTA_TIME;

  auto setup = collect_solve_setup();
  auto &var = setup.var;
  auto &var_groups = setup.var_groups;
  if (setup.nvars == 0)
    return;

  // Refill each recovered fine level's refinement-boundary (cf) ghosts.
  // Synchronized levels (and old/synchronized checkpoints with no restored band
  // data) use spatial tl=0 prolongation. Unsynchronized levels carry mid-cycle
  // dense-output state in their restored consumer bands and reconstruct the
  // cf-ghosts the uninterrupted run's previous fine substep last wrote.
  if (var_groups.size() > 0) {
    var.check_valid(make_valid_int(),
                    "ODESolvers_Solve_Subcycling_Recovery requires the tl=0 "
                    "interior to be populated by checkpoint recovery");

    // Spatial prolongation fills every fine level's cf-ghosts; the
    // unsynchronized levels below are then overwritten with dense output.
    SyncGroupsByDirIProlongateOnly(cctkGH, var_groups.size(), var_groups.data(),
                                   nullptr, /*tl=*/0);

    active_levels->loop_coarse_to_fine([&](auto &restrict leveldata) {
      const int level = leveldata.level;
      if (level == 0)
        return;
      const auto &patchdata = ghext->patchdata.at(leveldata.patch);
      const auto &prev_leveldata = patchdata.leveldata.at(level - 1);
      // Time-aligned with the parent: spatial prolongation above is correct.
      if (leveldata.iteration == prev_leveldata.iteration)
        return;
      // Reconstruct only where restored consumer-band data is present (a band
      // rebuilt but never read falls back to the spatial path).
      bool have_bands = false;
      for (const int gi : var_groups) {
        const auto &groupdata = *leveldata.groupdata.at(gi);
        if (groupdata.ks_consumer_band[0] &&
            !groupdata.ks_consumer_band[0]->empty()) {
          have_bands = true;
          break;
        }
      }
      if (!have_bands)
        return;
      const CCTK_REAL xsi = 0.5;
      if (CCTK_EQUALS(method, "IMEX42L") || CCTK_EQUALS(method, "IMEX32L")) {
        // IMEX checkpoints store the parent's accepted new endpoint in band
        // slot zero, so recovery uses the same convex endpoint interpolation
        // as the uninterrupted solve.
        Subcycling::CalcYfFromEndpoints_MFlevel(leveldata, var_groups,
                                                /*Yf_tl=*/0, xsi);
      } else if (ghext->num_rk_stages == 3) {
        Subcycling::CalcYfFromKcs_MFlevel<3>(leveldata, var_groups, /*Yf_tl=*/0,
                                             dt * 2, xsi, /*stage0=*/1);
      } else {
        Subcycling::CalcYfFromKcs_MFlevel<4>(leveldata, var_groups, /*Yf_tl=*/0,
                                             dt * 2, xsi, /*stage0=*/1);
      }
    });
    synchronize();
    var.set_valid(make_valid_all());
  }
}

extern "C" void ODESolvers_CheckTimelevels(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_ODESolvers_CheckTimelevels;
  DECLARE_CCTK_PARAMETERS;

  for (int gi = 0; gi < CCTK_NumGroups(); ++gi) {
    if (get_group_rhs(gi) < 0)
      continue; // not an ODE-evolved group
    const int ntls = CCTK_ActiveTimeLevelsGI(cctkGH, gi);
    if (ntls >= 2)
      CCTK_VERROR("ODESolvers subcycling requires evolved groups to have a "
                  "single timelevel, but group \"%s\" has %d active "
                  "timelevels. Subcycling does not support timelevels >= 2 "
                  "for evolution variables.",
                  CCTK_FullGroupName(gi), ntls);
  }
}

} // namespace ODESolvers
