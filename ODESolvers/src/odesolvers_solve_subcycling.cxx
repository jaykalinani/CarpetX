#include "rk_methods.hxx"
#include "solve.hxx"
#include "IMEX32L_coeffs.hpp"

// Driver primitives for the state-based refinement-boundary fill
// (StoreRKOldState, StoreRKStage, FillRKBoundary).
// TODO: Don't include files from other thorns; create a proper interface.
#include "../../CarpetX/src/subcycling.hxx"

#include <AMReX_MultiFab.H>

#include <array>

namespace ODESolvers {

namespace {

struct solve_setup_t {
  statecomp_t var, rhs;
  std::vector<int> var_groups, rhs_groups, dep_groups;
  int nvars = 0;
};

// Where within the parent's step the dense-output polynomial is evaluated for
// a fine level's refinement-boundary fill. This is the only place the
// evaluation-point convention lives.
struct dense_output_point_t {
  CCTK_REAL xsi;
  int stage;
};

// Because Evolve advances the fine counter before CCTK_EVOL, a fine level is
// aligned with its parent exactly during its second substep (it lands on the
// parent's time), hence xsi = 1/2; during the first substep it is behind,
// hence xsi = 0. At the virtual end-of-substep stage (num_rk_stages + 1) the
// polynomial is evaluated at xsi + 1/2 with the pure-U (stage 1) combination,
// which is what a normal RK step would need at its next stage 1.
dense_output_point_t
dense_output_point(const CarpetX::GHExt::PatchData::LevelData &leveldata,
                   const int stage) {
  assert(leveldata.level > 0);
  const auto &patchdata = CarpetX::ghext->patchdata.at(leveldata.patch);
  const auto &prev_leveldata = patchdata.leveldata.at(leveldata.level - 1);
  const int virtual_end = CarpetX::ghext->num_rk_stages + 1;
  assert(stage >= 1 && stage <= virtual_end);
  const CCTK_REAL xsi =
      (leveldata.iteration == prev_leveldata.iteration) ? 0.5 : 0.0;
  if (stage == virtual_end)
    return {xsi + 0.5, 1};
  return {xsi, stage};
}

// Collect evolved groups into statecomp_t bundles. The old-state anchor is now
// a scratch copy of var(tl=0) made by the solver (no extra timelevel); the RK
// k-stages live as coarse-fine bands on the child level's GroupData. Operates
// on CarpetX::active_levels; no cGH needed.
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

  if (CCTK_EQUALS(method, "RK4-2") || CCTK_EQUALS(method, "RK4-3") ||
      CCTK_EQUALS(method, "RK42-IMEX") || CCTK_EQUALS(method, "RK42-IMEX2L"))
    CCTK_VERROR("ODESolvers method \"%s\" keeps RHS history in extra time "
                "levels and is not implemented for CarpetX::use_subcycling. "
                "Set Driver::use_subcycling = no.",
                method);

  // method and rhs_timelevels are steerable. Allocate before this step
  // caches MultiFab pointers. Multistep methods have already been refused.
  setup_rhs_storage(method, rhs_timelevels, verbose, use_subcycling);

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

  const CCTK_REAL saved_time = cctkGH->cctk_time;
  const CCTK_REAL old_time = cctkGH->cctk_time - dt;

  static Timer timer_lincomb("ODESolvers::Solve::lincomb");
  static Timer timer_rhs("ODESolvers::Solve::rhs");
  static Timer timer_poststep("ODESolvers::Solve::poststep");

  // The method's row in the RK method table (rk_methods.hxx): its stage
  // count and the effective weight b_s of each stage in the final update,
  //   y1 = y0 + dt * sum_s b_s f(Y_s),
  // handed to the driver's flux register so that after a full step it holds
  // exactly the flux combination the state received. ODESolvers_CheckMethod
  // rejected at PARAMCHECK any method this solver does not implement.
  const rk_method_t &rk = rk_method(method);
  assert(rk.subcycling_ok);

  const auto calcrhs = [&](const int n) {
    Interval interval_rhs(timer_rhs);
    if (verbose)
      CCTK_VINFO("Calculating RHS #%d at t=%g", n, double(cctkGH->cctk_time));
    CallScheduleGroup(cctkGH, "ODESolvers_RHS");
    rhs.check_valid(make_valid_int(),
                    "ODESolvers after calling ODESolvers_RHS");
    // Feed the driver's flux registers with this stage's flux, weighted by
    // the stage's effective weight and this batch's step, now: the flux
    // groups hold exactly the flux the update below consumes, and calcupdate
    // marks them invalid afterwards (they are dependents of the state).
    active_levels->loop_coarse_to_fine([&](const auto &restrict leveldata) {
      CarpetX::AccumulateFluxes(leveldata.patch, leveldata.level, n,
                                rk.b.at(n - 1) * dt);
    });
    synchronize();
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
  // Fill the refinement-boundary ghosts of var(tl=0) on every fine level for
  // the given stage: the driver evaluates the dense-output polynomial on the
  // level's own source bands (the parent's old state + k-stages) at
  // (stage0, xsi) and prolongates that single coarse state in space.
  // dtc = dt*2 is the parent's step under 2:1 time refinement.
  const auto calcys_rmbnd = [&](const int stage) {
    if (verbose)
      CCTK_VINFO(
          "Fill refinement boundary ghost zones using Ys for stage #%d at t=%g",
          stage, double(cctkGH->cctk_time));

    active_levels->loop_coarse_to_fine([&](auto &leveldata) {
      if (leveldata.level == 0)
        return;
      const auto [xsi, stage0] = dense_output_point(leveldata, stage);
      CarpetX::FillRKBoundary(leveldata.patch, leveldata.level, var_groups,
                              /*tl=*/0, stage0, xsi, dt * 2);
    });
    synchronize();
    var.set_valid(make_valid_all());
  };
  // Store the interior RHS of this stage of each level into the k-stage source
  // band of its child level, which owns the bands (levels with children only),
  // to be combined into the children's refinement-boundary fills by
  // calcys_rmbnd.
  const auto setks = [&](const int stage) {
    if (verbose)
      CCTK_VINFO(
          "Set interior Ks for stage #%d at t=%g, to be prolongated later",
          stage, double(cctkGH->cctk_time));
    active_levels->loop_coarse_to_fine([&](const auto &restrict leveldata) {
      CarpetX::StoreRKStage(leveldata.patch, leveldata.level, var_groups,
                            rhs_groups, stage);
    });
    synchronize();
  };
  // Capture u(t_n) = var(tl=0) of each level into the old_source_band of its
  // child level, which owns the bands (interior only, levels with children
  // only), once per step before the RK stages overwrite var. This is also
  // where the child's RK buffers are (lazily) allocated: that reads the
  // next-finer level, so it must run once all levels exist, and it warms an
  // AMReX cache and opens its own MFIter/OpenMP region, so it must run
  // single-threaded (loop_serially).
  const auto store_old = [&]() {
    active_levels->loop_serially([&](const auto &restrict leveldata) {
      CarpetX::StoreRKOldState(leveldata.patch, leveldata.level, var_groups,
                               /*tl=*/0);
    });
    synchronize();
  };

  // IMEX methods have no compatible RK dense-output formula here. Store the
  // accepted coarse-step secant in every stage band; the existing RK3/RK4
  // dense-output polynomial then becomes exact linear endpoint interpolation
  // at the IMEX32L/IMEX42L stage abscissae.
  const auto refresh_parent_endpoint_stages = [&]() {
    active_levels->loop_serially([&](const auto &restrict leveldata) {
      if (leveldata.level == 0)
        return;
      CarpetX::StoreRKEndpointStages(leveldata.patch, leveldata.level - 1,
                                     var_groups, /*tl=*/0, dt * 2);
    });
    synchronize();
  };
  const auto store_endpoint_stages = [&]() {
    active_levels->loop_serially([&](const auto &restrict leveldata) {
      CarpetX::StoreRKEndpointStages(leveldata.patch, leveldata.level,
                                     var_groups, /*tl=*/0, dt);
    });
    synchronize();
  };

  // c_exp and c_imp are the explicit and implicit abscissae. They are equal
  // for IMEX42L. IMEX-SSP3(3,3,2) evaluates the flux and the source of a
  // stage at different times, so the two vectors differ there.
  const auto run_imex =
      [&](const vector<CCTK_REAL> &c_exp, const vector<CCTK_REAL> &c_imp,
          const vector<vector<CCTK_REAL> > &a_exp,
          const vector<vector<CCTK_REAL> > &a_imp,
          const vector<CCTK_REAL> &b_exp, const vector<CCTK_REAL> &b_imp) {
        const int nstages = c_exp.size();
        assert(int(c_imp.size()) == nstages);
        assert(nstages == rk.nstages);
        assert(int(a_exp.size()) == nstages);
        assert(int(a_imp.size()) == nstages);
        assert(int(b_exp.size()) == nstages);
        assert(int(b_imp.size()) == nstages);

        const auto old = var.copy(make_valid_all());
        vector<statecomp_t> fks;
        vector<statecomp_t> gks;
        fks.reserve(nstages);
        gks.reserve(nstages);

        if (!var_groups.empty()) {
          store_old();
          refresh_parent_endpoint_stages();
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
              old_time + c_imp.at(stage) * dt;
        };

        for (int stage = 0; stage < nstages; ++stage) {
          if (stage > 0) {
            define_stage_base(stage);
            calcys_rmbnd(stage + 1);
            calcpreimplicit();
            // PostStep routines may update evolved interiors (for example,
            // conservative-to-primitive atmosphere repair) and their regular
            // subcycling SYNC does not refill coarse-fine ghosts during a fine
            // substep. Restore the time-interpolated refinement boundary
            // before the diagonal implicit solve consumes the stage state.
            calcys_rmbnd(stage + 1);
          } else {
            *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) =
                old_time + c_imp.at(0) * dt;
            // The first implicit node of IMEX-SSP3(3,3,2) is gamma, not 0.
            // FillRKBoundary is keyed by stage, not by an abscissa.
            if (c_imp.at(0) != 0)
              calcys_rmbnd(1);
          }

          const CCTK_REAL diag = a_imp.at(stage).at(stage);
          statecomp_t g;
          if (diag == 0) {
            calcimplicitrhs(stage + 1);
            CallScheduleGroup(cctkGH, "ODESolvers_AfterImplicitRHS");
            g = rhs.copy(make_valid_int());
          } else {
            const auto base = var.copy(make_valid_all());
            calcimplicitstep(stage + 1, diag * dt);

            // Recover g(Y_i) before PostStep applies any numerical projection.
            statecomp_t::lincomb(
                rhs, 0.0,
                vector<CCTK_REAL>{1.0 / (diag * dt), -1.0 / (diag * dt)},
                vector<const statecomp_t *>{&var, &base}, make_valid_int());
            rhs.check_valid(make_valid_int(),
                            "ODESolvers effective implicit RHS");
            CallScheduleGroup(cctkGH, "ODESolvers_AfterImplicitRHS");
            g = rhs.copy(make_valid_int());

            calcys_rmbnd(stage + 1);
            calcpoststep();
          }

          if (c_exp.at(stage) != c_imp.at(stage)) {
            *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) =
                old_time + c_exp.at(stage) * dt;
            calcys_rmbnd(stage + 1);
          }
          calcrhs(stage + 1);
          fks.push_back(rhs.copy(make_valid_int()));
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
        calcys_rmbnd(nstages + 1);
        calcpoststep();
        if (!var_groups.empty())
          store_endpoint_stages();
      };

  *const_cast<CCTK_REAL *>(&cctkGH->cctk_time) = old_time;

  if (CCTK_EQUALS(method, "constant")) {

    // y1 = y0

    // do nothing

  } else if (CCTK_EQUALS(method, "RK4") ||
             CCTK_EQUALS(method, "semi-implicit")) {

    // k1 = f(y0)
    // k2 = f(y0 + h/2 k1)
    // k3 = f(y0 + h/2 k2)
    // k4 = f(y0 + h k3)
    // y1 = y0 + h/6 k1 + h/3 k2 + h/3 k3 + h/6 k4

    // The table row must describe the sequence below
    assert(rk.nstages == 4);

    // Scratch copy of u(t_n) = var(tl=0), the RK4 interior anchor y0. At one
    // timelevel var(tl=0) holds the previous step's result, so no init copy is
    // needed (mirrors the non-subcycling solver).
    const auto old = var.copy(make_valid_all());

    // Capture u(t_n) into the old source bands (and allocate the bands) before
    // the RK stages overwrite var.
    if (var_groups.size() > 0)
      store_old();
    if (CCTK_EQUALS(method, "semi-implicit") && !var_groups.empty())
      refresh_parent_endpoint_stages();

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

    if (CCTK_EQUALS(method, "semi-implicit")) {
      calcimplicitstep(1, dt);
      calcpoststep();
      if (!var_groups.empty())
        store_endpoint_stages();
    }

    // No calcys_rmbnd(1) here: refinement-boundary ghosts are kept aligned by
    // subcycling-aware POSTRESTRICT SYNCs. The post-recovery case is handled
    // by ODESolvers_Solve_Subcycling_Recovery at CCTK_CPINITIAL.

  } else if (CCTK_EQUALS(method, "SSPRK3")) {

    // k1 = f(y0)
    // k2 = f(y0 + h k1)
    // k3 = f(y0 + h/4 k1 + h/4 k2)
    // y1 = y0 + h/6 k1 + h/6 k2 + 2/3 h k3

    // The table row must describe the sequence below
    assert(rk.nstages == 3);

    // Scratch copy of u(t_n) = var(tl=0), the SSPRK3 interior anchor y0.
    const auto old = var.copy(make_valid_all());

    // Capture u(t_n) into the old source bands (and allocate the bands) before
    // the RK stages overwrite var.
    if (var_groups.size() > 0)
      store_old();

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
    run_imex(c, c, a_exp, a_imp, b, b);

  } else if (CCTK_EQUALS(method, "IMEX32L")) {

    // Pareschi and Russo, J. Sci. Comput. 25 (2005) 129-155, Table 5.
    assert(rk.nstages == 3);
    vector<CCTK_REAL> c_exp(3), c_imp(3), b(3);
    vector<vector<CCTK_REAL> > a_exp(3, vector<CCTK_REAL>(3));
    vector<vector<CCTK_REAL> > a_imp(3, vector<CCTK_REAL>(3));
    for (int i = 0; i < 3; ++i) {
      c_exp.at(i) = imex32l_c_exp<CCTK_REAL>(i);
      c_imp.at(i) = imex32l_c_imp<CCTK_REAL>(i);
      b.at(i) = imex32l_b<CCTK_REAL>(i);
      for (int j = 0; j < 3; ++j) {
        a_exp.at(i).at(j) = imex32l_a_exp<CCTK_REAL>(i, j);
        a_imp.at(i).at(j) = imex32l_a_imp<CCTK_REAL>(i, j);
      }
    }
    run_imex(c_exp, c_imp, a_exp, a_imp, b, b);

  } else {
    // Unreachable: rk.subcycling_ok is asserted above, and the table marks
    // only the methods implemented here as subcycling_ok
    CCTK_VERROR("ODESolvers::method = \"%s\" is marked subcycling_ok in "
                "rk_methods.hxx but has no branch in the subcycling solver",
                method);
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
               "from the level's restored source bands otherwise)");

  static Timer timer("ODESolvers::Solve_Subcycling_Recovery");
  Interval interval(timer);

  const CCTK_REAL dt = CCTK_DELTA_TIME;

  auto setup = collect_solve_setup();
  auto &var = setup.var;
  auto &var_groups = setup.var_groups;
  if (setup.nvars == 0)
    return;

  // Refill each recovered fine level's refinement-boundary (cf) ghosts.
  // Time-aligned levels use spatial tl=0 prolongation. A level that is behind
  // its parent is mid-cycle: its restored source bands hold the parent's
  // in-progress coarse step, and the same driver fill the uninterrupted run
  // made at the previous fine substep's virtual end-of-step reconstructs the
  // cf-ghosts it last wrote. The checkpoint reader has already refused a
  // mid-cycle checkpoint that lacks those bands.
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
      // Mirror the previous fine substep's calcys_rmbnd at the virtual
      // end-of-step: base offset 0.0 plus the +0.5 give xsi = 0.5, stage0 = 1,
      // dtc = dt*2 (the parent's step).
      CarpetX::FillRKBoundary(leveldata.patch, level, var_groups, /*tl=*/0,
                              /*stage=*/1, /*xsi=*/0.5, dt * 2);
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

// Reject at PARAMCHECK, instead of at the first CCTK_EVOL, a method the
// configuration cannot run (rk_methods.hxx):
//  - under subcycling, one the subcycling solver does not implement; only
//    the implemented ones have a stage count that fits the driver's band
//    arrays and a b row the flux registers can be fed with;
//  - in a run that may refine (CarpetX::max_num_levels > 1), an implicit
//    (IMEX) method when some group gets a flux register: the driver would
//    allocate the register on every refined level, and only an explicit
//    method hands every stage's flux to it with a known weight
//    (ODESolvers_Solve's calcrhs). A unigrid run owns no register (they
//    live on levels > 0), so an implicit method is fine there even with
//    CarpetX::do_reflux = yes.
// Runs after WRAGH, where ODESolvers_InitConstants publishes
// rk_integrated_group, so CarpetX::group_has_flux_register (the predicate
// that register allocation uses) already gives its final answer.
extern "C" void ODESolvers_CheckMethod(CCTK_ARGUMENTS) {
  DECLARE_CCTK_ARGUMENTS_ODESolvers_CheckMethod;
  DECLARE_CCTK_PARAMETERS;

  const rk_method_t &rk = rk_method(method);

  if (CarpetX::ghext->use_subcycling && !rk.subcycling_ok) {
    std::string supported;
    for (const auto &row : rk_methods)
      if (row.subcycling_ok)
        supported +=
            (supported.empty() ? "\"" : ", \"") + std::string(row.name) + "\"";
    CCTK_VERROR("ODESolvers::method = \"%s\" is not supported by the "
                "subcycling solver (CarpetX::use_subcycling = yes); supported "
                "methods: %s",
                method, supported.c_str());
  }

  // CarpetX::max_num_levels is private to the driver; read it by name
  int max_num_levels_type;
  const void *const max_num_levels_p =
      CCTK_ParameterGet("max_num_levels", "CarpetX", &max_num_levels_type);
  assert(max_num_levels_p);
  assert(max_num_levels_type == PARAMETER_INT);
  const CCTK_INT max_num_levels =
      *static_cast<const CCTK_INT *>(max_num_levels_p);

  if (CarpetX::ghext->do_reflux && !rk.reflux_ok && max_num_levels > 1) {
    for (int gi = 0; gi < CCTK_NumGroups(); ++gi) {
      if (!CarpetX::group_has_flux_register(gi))
        continue;
      CCTK_VERROR("CarpetX::do_reflux with fluxes= tags and "
                  "CarpetX::max_num_levels > 1 requires an explicit "
                  "ODESolvers::method, but \"%s\" is implicit and group "
                  "\"%s\" (integrated by ODESolvers) carries a fluxes= tag. "
                  "Choose an explicit method or set CarpetX::do_reflux = no.",
                  method, CCTK_FullGroupName(gi));
    }
  }
}

} // namespace ODESolvers
