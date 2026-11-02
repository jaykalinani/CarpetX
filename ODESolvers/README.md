# ODESolvers

| Author(s)      | Erik Schnetter and Liwei Ji |
|:---------------|:----------------------------|
| Maintainer(s)  | Erik Schnetter and Liwei Ji |
| Licence        | LGPL |


## Purpose

Solve systems of coupled ordinary differential equations


## Tag requirements

A group with an `rhs=` tag is integrated by ODESolvers. ODESolvers checks its tags at startup (`CCTK_PARAMCHECK`) and aborts with an error naming the group if they are not allowed:

- It must be checkpointed, i.e. it must not set `checkpoint="no"`, whatever its `evolve` tag. A checkpoint would omit its state and, under subcycling, its Runge-Kutta source bands, so recovery could not restore it. Enforced by `ODESolvers_CheckTags`, which tests this rule first.
- It must be evolved, i.e. it must not set `evolve="no"`. Regrid would not reliably refill its state. Enforced by `ODESolvers_CheckTags`.
- Under subcycling (`CarpetX::use_subcycling = yes`) it must also have a single timelevel. Enforced by `ODESolvers_CheckTimelevels`.

The default tags (no `checkpoint` or `evolve` tag) satisfy the first two rules.


## Subcycling

Add the following parameters to your parameter file

```
CarpetX::use_subcycling = yes
CarpetX::restrict_during_sync = no
```

Memory: under subcycling each refined level holds, per evolved group, two extra zero-ghost work buffers over its coarse-fine boundary footprint, one at the coarse and one at the fine resolution (the latter has about 8 times the cells of the former). They come on top of the `num_rk_stages + 1` coarse-resolution source bands that each refined level keeps per evolved group, which hold its parent's start-of-step state and stage derivatives on the same footprint. The bands and the two work buffers are allocated together, when the parent level first stores its old state after the refined level was made, and stay allocated until the next regrid remakes the level, so that the per-stage boundary fill allocates nothing. The bands of a group cover the footprint of that group's own prolongation stencil, so groups with different `prolongation_type` / `prolongation_order` tags may be mixed freely. Runs without subcycling never allocate any of these.


## IMEX methods

Both drivers use `src/imex_tableaux.hxx`. Implicit increments are recovered
before `PostStep` numerical projections. No extra final implicit projection
is applied. Flux registers use the same explicit weights as the accepted state.

- `IMEX42L` retains Table 10 of [Izquierdo et al.](https://arxiv.org/html/2211.00027v2).
  The explicit part is RK4; the additive scheme is second order. Its implicit
  part is L-stable, but that alone is not a stiff coupled-transport guarantee.
- `IMEX32L` now uses the L-stable IMEX-SSP3(3,3,2) in Table 5 of
  [Pareschi and Russo](https://arxiv.org/pdf/1009.2757). Its explicit part is
  SSPRK3, and its implicit diagonal is `gamma=1-1/sqrt(2)` at all three stages.
  Explicit times `(0,1,1/2)` differ from implicit times `(gamma,1-gamma,1/2)`;
  each partition's time-dependent fields and boundaries are refreshed there.
  **This deliberately changes results from the former IMEX32L.** The old
  printed CK tableau has `b.A_I.1=5/6`, not `1/2`, and is only first order
  for pure implicit decay. Matching the printed table was not sufficient.
- `IMEX122` is explicit/implicit midpoint. The second RHS is evaluated at the
  midpoint and the implicit increment is scaled exactly once. It is second
  order and A-stable, not L-stable, and is not supported under subcycling.
  `Implicit Euler` remains a deprecated compatibility alias with a startup
  warning: it does not mean backward Euler.

Under subcycling, IMEX coarse-fine data use accepted parent endpoint secants.
Both partition times, including the first nonzero implicit stage, are evaluated
at their actual parent-step fractions. This is linear time interpolation, not
a new high-order IMEX dense-output formula.

For the scalar split `y'=(lambda_E+lambda_I)*y`, at fixed `z_E=lambda_E*dt`,
the unprepared stiff limit `z_I -> -infinity` is zero for corrected `IMEX32L`,
but `-z_E/2` for `IMEX42L`. Midpoint's pure implicit amplification tends to
`-1`. Do not equate pure-implicit L-stability, nonautonomous order, prepared
diffusion accuracy, or nonlinear radiation-matter stability: test each.

Corrected `IMEX32L` is not globally stiffly accurate. In a relaxation problem,
its internal stages approach the equilibrium manifold and its conserved
variables approach the explicit reduced-system update, but the accepted fast
variables need not lie exactly on that manifold. The next first implicit stage
relaxes them again. This distinction matters when examining output flux
speckles; it is not permission to append an unweighted implicit projection.

Run the source-reading math checks without a Cactus build:

```
python3.11 ODESolvers/test/test_imex_tableaux.py
```

They check the actual shared coefficients, additive order, scalar autonomous
and time-dependent convergence, stiff limits, and partition-time/source
contracts; all 12 currently pass. They do not execute the compiled schedules.
The paired nuX/ODESolvers CUDA build, short source-only relaxation runs and
a four-rank/four-GPU two-step IMEX42L TOV smoke have also completed. These
checks do not establish all-method AMR order, nonlinear diffusion accuracy
or long coupled TOV stability; those validations remain required.
