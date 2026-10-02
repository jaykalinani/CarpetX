# ODESolvers

| Author(s)      | Erik Schnetter and Liwei Ji |
|:---------------|:----------------------------|
| Maintainer(s)  | Erik Schnetter and Liwei Ji |
| Licence        | LGPL |


## Purpose

Solve systems of coupled ordinary differential equations


## RK4-2 and RK4-3

`RK4-2` and `RK4-3` are the multistep Runge-Kutta methods from
arXiv:2603.05763. They keep previous RHS evaluations in time levels of the
RHS groups, so each RHS group needs `TIMELEVELS=4` in its `interface.ccl`.
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
This is not an order-4 implicit method.

## Subcycling

Add the following parameters to your parameter file

```
CarpetX::use_subcycling = yes
CarpetX::restrict_during_sync = no
```


## To Do

Implement IMEX methods as e.g. described in

Ascher, Ruuth, Spiteri: "Implicit-Explicit Runge-Kutta Methods for
Time-Dependent Partial Differential Equations", Appl. Numer. Math 25
(1997), pages 151-167,
<http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.48.1525&rep=rep1&type=pdf>.
