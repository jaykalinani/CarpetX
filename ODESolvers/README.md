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

## RK4-2 and RK4-3

`RK4-2` and `RK4-3` are the multistep Runge-Kutta methods from
arXiv:2603.05763. They keep previous RHS evaluations in time levels of the
RHS groups. `ODESolvers::rhs_timelevels = 0` allocates the minimum the
current method needs: 2 for `RK4-2`, `RK42-IMEX`, and `RK42-IMEX2L`, 3 for
`RK4-3`, and 1 for every one-step method. Each RHS group still declares
`TIMELEVELS=4` in its `interface.ccl`, which is the ceiling, because
`ODESolvers::method` is steerable. A positive `rhs_timelevels` must be at
least that minimum and at most the declaration.
`RK4_dash_2_sol` selects published solution (1) (the default) or (2). The
first steps, and any step whose history was invalidated by regridding or
recovery, take a classic RK4 step to fill that history. These methods are
not available when `CarpetX::use_subcycling` is `yes`.

`RK42-IMEX` is the same RK4-2(1) tableau on the explicit RHS (fluxes, and
whatever else `ODESolvers_RHS` evaluates), with one backward-Euler source
step after each positive stage abscissa and after the final update. The
source step is `ODESolvers_ImplicitStep`, which nuX implements in
`nuX_M1_CalcUpdate`. Solution (2) is rejected: its first abscissa is
negative. A negative abscissa keeps the explicit update and, when
`nuX_Base::rF` is an evolved group, puts the momentum back to the last
relaxed stage. The step that fills a missing history is IMEX Euler, not
classic RK4, because the explicit RHS does not contain the collision term.
This is not an order-4 implicit method. Its source step is backward Euler,
so the source part is first order.

`RK42-IMEX2L` keeps those same RK4-2(1) flux weights and replaces the
source step with a two-stage L-stable DIRK (`gamma = 1 - sqrt(2)/2`).
Both implicit solves use step `gamma*dt`. The source tendency from the
first solve is added into the later flux stages, including the negative
abscissa, which is a weight and not a solve. There is no momentum
restore. The finished step is the second solve. A missing history takes
one classic RK4 flux step coupled to that same DIRK. Later steps are
RK4-2 on the flux. Those steps match RK4 and RK4-2 when the implicit
kernel leaves its saved base unchanged, so the recovered source is zero.
nuX floors and projects the radiation in the pre-implicit PostStep for
this method only, which keeps that repair out of the source tendency.
IMEX42L instead rebuilds the step from the recovered source, so the same
repair stays inside its source. The source part is order 2. This is not
an order-4 implicit method, and subcycling is refused.

## IMEX32L

`IMEX32L` is the IMEX-SSP3(3,3,2) scheme in Table 5 of Pareschi and
Russo, "Implicit-Explicit Runge-Kutta Schemes and Applications to
Hyperbolic Systems with Relaxation," Journal of Scientific Computing
25 (2005) 129–155
(<https://doi.org/10.1007/s10915-004-4636-4>, arXiv:1009.2757). The
explicit part is SSPRK3, with nodes \(0\), \(1\), and \(1/2\). The
implicit part is an L-stable DIRK with diagonal
\(\gamma = 1 - 1/\sqrt{2}\) on every stage, so each step takes three
implicit solves of size \(\gamma\Delta t\). The implicit nodes are
\(\gamma\), \(1-\gamma\), and \(1/2\). The flux and the source of one
stage are evaluated at those two different times. The weights are
\(1/6\), \(1/6\), and \(2/3\). The finished step is that weighted
combination, so a repair made inside the implicit solve stays in the
recovered source. The combined scheme is order 2. With the source absent
it is SSPRK3, which is order 3.

## Subcycling

Add the following parameters to your parameter file

```
CarpetX::use_subcycling = yes
CarpetX::restrict_during_sync = no
```

Memory: under subcycling each refined level holds, per evolved group, two extra zero-ghost work buffers over its coarse-fine boundary footprint, one at the coarse and one at the fine resolution (the latter has about 8 times the cells of the former). They come on top of the `num_rk_stages + 1` coarse-resolution source bands that each refined level keeps per evolved group, which hold its parent's start-of-step state and stage derivatives on the same footprint. The bands and the two work buffers are allocated together, when the parent level first stores its old state after the refined level was made, and stay allocated until the next regrid remakes the level, so that the per-stage boundary fill allocates nothing. The bands of a group cover the footprint of that group's own prolongation stencil, so groups with different `prolongation_type` / `prolongation_order` tags may be mixed freely. Runs without subcycling never allocate any of these.


## To Do

Implement IMEX methods as e.g. described in

Ascher, Ruuth, Spiteri: "Implicit-Explicit Runge-Kutta Methods for
Time-Dependent Partial Differential Equations", Appl. Numer. Math 25
(1997), pages 151-167,
<http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.48.1525&rep=rep1&type=pdf>.
