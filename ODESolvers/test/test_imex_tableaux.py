#!/usr/bin/env python3
"""Read the production tableaux; check scalar math without building Cactus.

Run: python3.11 ODESolvers/test/test_imex_tableaux.py
These checks do not substitute for a compiled CPU/GPU or AMR evolution test.
"""

import ast
import math
from fractions import Fraction
from pathlib import Path
import re
import unittest


THORN = Path(__file__).resolve().parents[1]


def _expression(node, source, names):
    if isinstance(node, ast.List):
        return [_expression(item, source, names) for item in node.elts]
    if isinstance(node, ast.Constant):
        return Fraction(ast.get_source_segment(source, node))
    if isinstance(node, ast.Name):
        return names[node.id]
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -_expression(node.operand, source, names)
    if isinstance(node, ast.BinOp):
        left = _expression(node.left, source, names)
        right = _expression(node.right, source, names)
        operations = {ast.Add: lambda: left + right,
                      ast.Sub: lambda: left - right,
                      ast.Mult: lambda: left * right,
                      ast.Div: lambda: left / right}
        return operations[type(node.op)]()
    raise ValueError(f"Unexpected coefficient syntax: {ast.dump(node)}")


def read_tableaux():
    source = (THORN / "src/imex_tableaux.hxx").read_text()
    source = re.sub(r"//[^\n]*", "", source)
    gamma = re.search(r"imex_gamma\s*=\s*([\d.]+)\s*;", source)
    names = {"imex_gamma": Fraction(gamma.group(1))}
    result = {}
    for name in ("imex32l", "imex42l", "imex122"):
        start = source.index("{", source.index(f"imex_tableau_t {name}"))
        depth = 1
        end = start + 1
        while depth:
            depth += (source[end] == "{") - (source[end] == "}")
            end += 1
        expression = source[start:end].replace("{", "[").replace("}", "]")
        fields = _expression(ast.parse(expression, mode="eval").body,
                             expression, names)
        stages, ce, ci, ae, ai, be, bi = fields
        # std::array<std::array<...>> has one extra aggregate brace pair.
        if len(ae) == 1:
            ae, ai = ae[0], ai[0]
        n = int(stages)
        result[name] = {"n": n, "ce": ce[:n], "ci": ci[:n],
                        "ae": [row[:n] for row in ae[:n]],
                        "ai": [row[:n] for row in ai[:n]],
                        "be": be[:n], "bi": bi[:n]}
    return result


def amplification(tableau, explicit, implicit):
    stages = []
    for i in range(tableau["n"]):
        base = 1 + sum((explicit * tableau["ae"][i][j] +
                        implicit * tableau["ai"][i][j]) * stages[j]
                       for j in range(i))
        stages.append(base / (1 - implicit * tableau["ai"][i][i]))
    return 1 + sum((explicit * be + implicit * bi) * stage
                   for be, bi, stage in zip(tableau["be"], tableau["bi"], stages))


def manufactured_step(tableau, state, time, step, a, b, p, q):
    explicit, implicit = [], []
    for i in range(tableau["n"]):
        te = time + float(tableau["ce"][i]) * step
        ti = time + float(tableau["ci"][i]) * step
        base = state + step * sum(float(tableau["ae"][i][j]) * explicit[j] +
                                  float(tableau["ai"][i][j]) * implicit[j]
                                  for j in range(i))
        diagonal = float(tableau["ai"][i][i]) * step
        stage = (base + diagonal * q * ti) / (1 - diagonal * b)
        explicit.append(a * stage + p * te)
        implicit.append(b * stage + q * ti)
    return state + step * sum(float(be) * f + float(bi) * g
                              for be, bi, f, g in zip(
                                  tableau["be"], tableau["bi"], explicit, implicit))


def relaxation_step(tableau, old_u, old_v, step, epsilon):
    # u'=-v, epsilon*v'=u-v. The reduced, conserved-variable equation is u'=-u.
    us, vs, gs = [], [], []
    for i in range(tableau["n"]):
        u = old_u - step * sum(tableau["ae"][i][j] * vs[j] for j in range(i))
        base_v = old_v + step * sum(tableau["ai"][i][j] * gs[j] for j in range(i))
        diagonal = step * tableau["ai"][i][i]
        v = (epsilon * base_v + diagonal * u) / (epsilon + diagonal)
        us.append(u)
        vs.append(v)
        gs.append((u - v) / epsilon)
    u = old_u - step * sum(w * v for w, v in zip(tableau["be"], vs))
    v = old_v + step * sum(w * g for w, g in zip(tableau["bi"], gs))
    return u, v, us, vs


class TestIMEX(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tableaux = read_tableaux()

    def test_additive_second_order_and_partition_times(self):
        for name, tableau in self.tableaux.items():
            with self.subTest(method=name):
                for matrix, times in (("ae", "ce"), ("ai", "ci")):
                    for row, time in zip(tableau[matrix], tableau[times]):
                        self.assertAlmostEqual(float(sum(row)), float(time), places=14)
                for weights in ("be", "bi"):
                    self.assertAlmostEqual(float(sum(tableau[weights])), 1, places=14)
                    for times in ("ce", "ci"):
                        self.assertAlmostEqual(float(sum(w * c for w, c in zip(
                            tableau[weights], tableau[times]))), 0.5, places=14)
                for i in range(tableau["n"]):
                    for j in range(tableau["n"]):
                        if j >= i:
                            self.assertEqual(tableau["ae"][i][j], 0)
                        if j > i:
                            self.assertEqual(tableau["ai"][i][j], 0)

    def test_canonical32_first_diagonal_and_distinct_times(self):
        tableau = self.tableaux["imex32l"]
        self.assertNotEqual(tableau["ci"][0], tableau["ce"][0])
        self.assertAlmostEqual(float(tableau["ci"][0]), 1 - 1 / math.sqrt(2))
        self.assertTrue(all(tableau["ai"][i][i] > 0 for i in range(3)))

    def test_explicit_orders(self):
        for name, order in (("imex32l", 3), ("imex42l", 4), ("imex122", 2)):
            with self.subTest(method=name):
                errors = []
                for n in (16, 32):
                    value = float(amplification(self.tableaux[name], Fraction(-1, n), 0)) ** n
                    errors.append(abs(value - math.exp(-1)))
                observed = math.log2(errors[0] / errors[1])
                self.assertLess(abs(observed - order), 0.12)

    def test_implicit_and_mixed_nonautonomous_order(self):
        # y'=a*y+p*t + b*y+q*t; actual explicit/implicit times differ for32L.
        for name, tableau in self.tableaux.items():
            for a, b, p, q in ((0, -1, 0, 0), (-0.3, -0.7, 0, 0),
                               (-0.3, -0.7, 0.4, -0.1), (0, -1, 0, 0.5)):
                with self.subTest(method=name, equation=(a, b, p, q)):
                    errors = []
                    k = a + b
                    exact = math.exp(k) + (p + q) * (math.expm1(k) - k) / (k * k)
                    for n in (32, 64):
                        value = 1.0
                        for i in range(n):
                            value = manufactured_step(tableau, value, i / n,
                                                      1 / n, a, b, p, q)
                        errors.append(abs(value - exact))
                    observed = math.log2(errors[0] / errors[1])
                    self.assertLess(abs(observed - 2), 0.13)

    def test_pure_implicit_stability(self):
        for name, tableau in self.tableaux.items():
            for z in (-Fraction(1, 100), -1, -10, -1000):
                self.assertLessEqual(abs(amplification(tableau, 0, z)), 1)
        stiff = -10**16
        for name in ("imex32l", "imex42l"):
            self.assertLess(abs(amplification(self.tableaux[name], 0, stiff)), Fraction(1, 10**12))
        # Midpoint does not damp an arbitrarily stiff decay mode.
        self.assertAlmostEqual(float(amplification(self.tableaux["imex122"], 0, stiff)), -1)

    def test_sampled_left_half_plane_stability(self):
        # A regression sample, not a numerical proof of A-stability.
        for name, tableau in self.tableaux.items():
            for real in (0, -1e-12, -0.01, -0.1, -1, -10, -100, -1e4):
                for imag in (0, 1e-12, 0.01, 0.1, 1, 3, 10, 100, 1e4):
                    with self.subTest(method=name, z=(real, imag)):
                        self.assertLessEqual(abs(amplification(
                            tableau, 0, complex(real, imag))), 1 + 1e-12)

    def test_mixed_stiff_limit_is_not_confused_with_pure_stability(self):
        stiff = -10**16
        for explicit in (Fraction(-1), Fraction(-1, 4), Fraction(1, 2)):
            self.assertLess(abs(amplification(self.tableaux["imex32l"], explicit, stiff)),
                            Fraction(1, 10**12))
            # Existing42L has a nonzero unprepared mixed stiff limit: do not
            # advertise its pure-implicit L-stability as a TOV/AP proof.
            self.assertAlmostEqual(float(amplification(self.tableaux["imex42l"], explicit, stiff)),
                                   float(-explicit / 2), places=12)

    def test_macro_ap_does_not_imply_final_equilibrium_projection(self):
        tableau = self.tableaux["imex32l"]
        h, epsilon = Fraction(1, 10), Fraction(1, 10**20)
        macro = amplification(tableau, -h, 0)
        for initial_v in (Fraction(1), Fraction(2)):
            u, v, us, vs = relaxation_step(tableau, Fraction(1), initial_v, h, epsilon)
            self.assertLess(abs(u - macro), Fraction(1, 10**15))
            self.assertLess(max(abs(ui - vi) for ui, vi in zip(us, vs)),
                            Fraction(1, 10**15))
            # Not globally stiffly accurate: accepted fast variables need not
            # lie on the equilibrium manifold, even though all stages do.
            # Guard against an undocumented extra final implicit projection.
            self.assertGreater(abs(v - u), Fraction(1, 1000))

    def test_endpoint_boundary_times(self):
        # In a fine substep, parent fraction = 0 or1/2, plus c*dtfine/dtparent.
        # For repeated endpoint-secant bands this is exact for linear data.
        for name in ("imex32l", "imex42l"):
            for offset in (Fraction(0), Fraction(1, 2)):
                for times in ("ce", "ci"):
                    for c in self.tableaux[name][times]:
                        fraction = offset + c / 2
                        self.assertGreaterEqual(fraction, 0)
                        self.assertLessEqual(fraction, 1)
                        self.assertEqual(2 + 3 * fraction, 2 + 3 * offset + Fraction(3, 2) * c)

    def test_driver_uses_common_tableaux_and_both_partition_times(self):
        for filename in ("odesolvers_solve.cxx", "odesolvers_solve_subcycling.cxx"):
            source = (THORN / "src" / filename).read_text()
            with self.subTest(driver=filename):
                for name in ("imex32l", "imex42l"):
                    self.assertIn(f"run_imex({name});", source)
                self.assertIn("tableau.c_imp.at(stage)", source)
                self.assertIn("tableau.c_exp.at(stage)", source)
                self.assertIn("stage > 0 || tableau.c_imp.at(stage) != 0", source)
                self.assertNotIn("kprime2", source)
                if filename == "odesolvers_solve_subcycling.cxx":
                    self.assertIn("xsi + c / 2", source)
        ordinary = (THORN / "src/odesolvers_solve.cxx").read_text()
        self.assertIn("run_imex(imex122);", ordinary)

    def test_explicit_flux_weights_come_from_shared_tableaux(self):
        source = (THORN / "src/rk_methods.hxx").read_text()
        for name in self.tableaux:
            self.assertIn(f"imex_explicit_weights({name})", source)
            self.assertIn(f"{name}.nstages", source)

    def test_diagonal_increment_precedes_explicit_projection(self):
        for filename in ("odesolvers_solve.cxx", "odesolvers_solve_subcycling.cxx"):
            source = (THORN / "src" / filename).read_text()
            loop = source[source.index("const auto run_imex"):]
            loop = loop[loop.index("for (int stage = 0;"):]
            implicit = loop.index("calcimplicitstep(stage + 1, diag * dt)")
            effective = loop.index("1.0 / (diag * dt)", implicit)
            explicit_time = loop.index("old_time + c_exp * dt", effective)
            explicit_rhs = loop.index("calcrhs(stage + 1", explicit_time)
            self.assertLess(implicit, effective)
            self.assertLess(effective, explicit_time)
            self.assertLess(explicit_time, explicit_rhs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
