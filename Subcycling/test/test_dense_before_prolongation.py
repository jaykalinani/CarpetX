#!/usr/bin/env python3
"""Focused algebraic and wiring regression for RK dense prolongation order."""

import math
import sys
from pathlib import Path


def minmod(a, b):
    if a * b <= 0.0:
        return 0.0
    return math.copysign(min(abs(a), abs(b)), a)


def centred(a, b):
    return 0.5 * (a + b)


def prolongate(coarse, slope):
    fine = []
    for i in range(1, len(coarse) - 1):
        limited = slope(coarse[i] - coarse[i - 1],
                        coarse[i + 1] - coarse[i])
        fine.extend((coarse[i] - 0.25 * limited,
                     coarse[i] + 0.25 * limited))
    return fine


def add_scaled(a, factor, b):
    return [x + factor * y for x, y in zip(a, b)]


def max_difference(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def dense_coefficients(rk_stages, xsi, stage):
    """Mirror the factored production coefficients."""
    r = 0.5
    xsi2 = xsi * xsi
    if rk_stages == 3:
        result = [xsi - (5.0 / 6.0) * xsi2,
                  (1.0 / 6.0) * xsi2,
                  (2.0 / 3.0) * xsi2]
        bt = [1.0 - (5.0 / 3.0) * xsi,
              (1.0 / 3.0) * xsi,
              (4.0 / 3.0) * xsi]
        btt = [-5.0 / 3.0, 1.0 / 3.0, 4.0 / 3.0]
        if stage == 2:
            result = [a + r * at for a, at in zip(result, bt)]
        elif stage == 3:
            result = [a + 0.5 * r * at + 0.25 * r * r * att
                      for a, at, att in zip(result, bt, btt)]
        return result

    assert rk_stages == 4
    xsi3 = xsi2 * xsi
    result = [xsi - 1.5 * xsi2 + (2.0 / 3.0) * xsi3,
              xsi2 - (2.0 / 3.0) * xsi3,
              xsi2 - (2.0 / 3.0) * xsi3,
              -0.5 * xsi2 + (2.0 / 3.0) * xsi3]
    bt = [1.0 - 3.0 * xsi + 2.0 * xsi2,
          2.0 * xsi - 2.0 * xsi2,
          2.0 * xsi - 2.0 * xsi2,
          -xsi + 2.0 * xsi2]
    btt = [-3.0 + 4.0 * xsi, 2.0 - 4.0 * xsi,
           2.0 - 4.0 * xsi, -1.0 + 4.0 * xsi]
    bttt = [4.0, -4.0, -4.0, 4.0]
    if stage == 2:
        result = [a + 0.5 * r * at for a, at in zip(result, bt)]
    elif stage in (3, 4):
        r2 = r * r
        r3 = r2 * r
        at = 0.5 * r if stage == 3 else r
        att = 0.25 * r2 if stage == 3 else 0.5 * r2
        attt = 0.0625 * r3 if stage == 3 else 0.125 * r3
        ak = -4.0 if stage == 3 else 4.0
        result = [a + at * da + att * dda + attt * ddda
                  for a, da, dda, ddda in zip(result, bt, btt, bttt)]
        result[2] += attt * ak
        result[1] -= attt * ak
    return result


def direct_dense_increment(rk_stages, xsi, stage, ks):
    """Independent direct U/Ut/Utt/Uttt form used before factorization."""
    r = 0.5
    xsi2 = xsi * xsi
    if rk_stages == 3:
        b = [xsi - (5.0 / 6.0) * xsi2,
             (1.0 / 6.0) * xsi2,
             (2.0 / 3.0) * xsi2]
        bt = [1.0 - (5.0 / 3.0) * xsi,
              (1.0 / 3.0) * xsi,
              (4.0 / 3.0) * xsi]
        btt = [-5.0 / 3.0, 1.0 / 3.0, 4.0 / 3.0]
        u = sum(a * k for a, k in zip(b, ks))
        if stage == 1:
            return u
        ut = sum(a * k for a, k in zip(bt, ks))
        if stage == 2:
            return u + r * ut
        utt = sum(a * k for a, k in zip(btt, ks))
        return u + 0.5 * r * ut + 0.25 * r * r * utt

    assert rk_stages == 4
    xsi3 = xsi2 * xsi
    b = [xsi - 1.5 * xsi2 + (2.0 / 3.0) * xsi3,
         xsi2 - (2.0 / 3.0) * xsi3,
         xsi2 - (2.0 / 3.0) * xsi3,
         -0.5 * xsi2 + (2.0 / 3.0) * xsi3]
    bt = [1.0 - 3.0 * xsi + 2.0 * xsi2,
          2.0 * xsi - 2.0 * xsi2,
          2.0 * xsi - 2.0 * xsi2,
          -xsi + 2.0 * xsi2]
    btt = [-3.0 + 4.0 * xsi, 2.0 - 4.0 * xsi,
           2.0 - 4.0 * xsi, -1.0 + 4.0 * xsi]
    bttt = [4.0, -4.0, -4.0, 4.0]
    u = sum(a * k for a, k in zip(b, ks))
    if stage == 1:
        return u
    ut = sum(a * k for a, k in zip(bt, ks))
    if stage == 2:
        return u + 0.5 * r * ut
    utt = sum(a * k for a, k in zip(btt, ks))
    uttt = sum(a * k for a, k in zip(bttt, ks))
    r2 = r * r
    r3 = r2 * r
    at = 0.5 * r if stage == 3 else r
    att = 0.25 * r2 if stage == 3 else 0.5 * r2
    attt = 0.0625 * r3 if stage == 3 else 0.125 * r3
    ak = -4.0 if stage == 3 else 4.0
    return u + at * ut + att * utt + attt * (
        uttt + ak * (ks[2] - ks[1]))


def check_rk3_rk4_ordering():
    old = [0.0, 1.0, 2.0, 3.0, 4.0]
    k1 = [0.0, -4.0, 0.0, -4.0, 0.0]

    for rk_stages in (3, 4):
        coefficients = dense_coefficients(rk_stages, xsi=0.0, stage=2)
        expected_a1 = 0.5 if rk_stages == 3 else 0.25
        assert coefficients[0] == expected_a1
        assert all(a == 0.0 for a in coefficients[1:])

        for name, slope, should_commute in (
                ("minmod", minmod, False),
                ("linear-centred", centred, True)):
            compose_first = prolongate(
                add_scaled(old, coefficients[0], k1), slope)
            prolongate_first = add_scaled(
                prolongate(old, slope), coefficients[0],
                prolongate(k1, slope))
            error = max_difference(compose_first, prolongate_first)
            if should_commute:
                assert error <= 32.0 * sys.float_info.epsilon
            else:
                assert error >= 0.25
            print(f"RK{rk_stages} {name}: max difference = {error:.17g}")


def check_coefficient_values():
    expected_at_zero = {
        (3, 1): (0.0, 0.0, 0.0),
        (3, 2): (1.0 / 2.0, 0.0, 0.0),
        (3, 3): (7.0 / 48.0, 1.0 / 48.0, 1.0 / 12.0),
        (4, 1): (0.0, 0.0, 0.0, 0.0),
        (4, 2): (1.0 / 4.0, 0.0, 0.0, 0.0),
        (4, 3): (3.0 / 32.0, 1.0 / 8.0, 1.0 / 16.0, -1.0 / 32.0),
        (4, 4): (3.0 / 16.0, 1.0 / 8.0, 1.0 / 4.0, -1.0 / 16.0),
    }
    endpoints = {
        3: (1.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0),
        4: (1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0),
    }
    tolerance = 16.0 * sys.float_info.epsilon
    for (rk_stages, stage), expected in expected_at_zero.items():
        actual = dense_coefficients(rk_stages, xsi=0.0, stage=stage)
        assert max_difference(actual, expected) <= tolerance
    for rk_stages, expected in endpoints.items():
        actual = dense_coefficients(rk_stages, xsi=1.0, stage=1)
        assert max_difference(actual, expected) <= tolerance

    for rk_stages in (3, 4):
        samples = [[1.0 if s == basis else 0.0
                    for s in range(rk_stages)]
                   for basis in range(rk_stages)]
        samples.append([(-1.0 if s % 2 else 1.0) * (s + 1.25)
                        for s in range(rk_stages)])
        for stage in range(1, rk_stages + 1):
            for xsi in (0.0, 0.25, 0.5, 1.0):
                coefficients = dense_coefficients(rk_stages, xsi, stage)
                for ks in samples:
                    factored = sum(a * k for a, k in zip(coefficients, ks))
                    direct = direct_dense_increment(
                        rk_stages, xsi, stage, ks)
                    scale = max(1.0, abs(factored), abs(direct))
                    assert abs(factored - direct) <= 128.0 * tolerance * scale
    print("RK3/RK4 coefficients: all stages and representative times pass")


def check_production_ordering():
    repo = Path(__file__).resolve().parents[2]
    source = (repo / "ODESolvers/src/odesolvers_solve_subcycling.cxx").read_text()
    dense = (repo / "Subcycling/src/subcycling.hxx").read_text()
    begin = source.index("void prolongate_dense_state(")
    end = source.index("// Collect evolved groups", begin)
    implementation = source[begin:end]
    compose = implementation.index("CalcDenseStateBand<RKSTAGES>")
    prolongate_once = implementation.index("FillPatch_ProlongateToBand")
    scatter = implementation.index("ScatterBandToCoarseFineGhosts")
    assert compose < prolongate_once < scatter
    assert implementation.count("FillPatch_ProlongateToBand") == 1
    assert "coarsegroupdata.ks_source_band[s]" in implementation
    coefficient_begin = dense.index("DenseOutputCoefficients(")
    coefficient_end = dense.index("/** Form one dense RK state", coefficient_begin)
    coefficient_source = dense[coefficient_begin:coefficient_end]
    for token in ("RKSTAGES == 3", "stage == 2", "stage == 3",
                  "stage == 4", "a[2] += attt * ak",
                  "a[1] -= attt * ak"):
        assert token in coefficient_source
    forward = source[:source.index(
        'extern "C" void ODESolvers_Solve_Subcycling_Recovery')]
    assert "CalcYfFromKcs_MFlevel" not in forward
    recovery = source[source.index(
        'extern "C" void ODESolvers_Solve_Subcycling_Recovery'):]
    require_history = recovery.index("recovered_source_bands")
    spatial = recovery.index("SyncGroupsByDirIProlongateOnly")
    restore_accepted = recovery.index("recovered_accepted_consumer_band")
    scatter_accepted = recovery.rindex("ScatterBandToCoarseFineGhosts")
    assert require_history < spatial < scatter_accepted
    assert restore_accepted < scatter_accepted
    assert "/*prescribe_valid_cf_interface=*/false" in recovery
    assert "CalcYfFromKcs_MFlevel" not in recovery
    print("production wiring: dense source composition precedes one prolongation")


def check_checkpoint_contract():
    repo = Path(__file__).resolve().parents[2]
    driver = (repo / "CarpetX/src/driver.cxx").read_text()
    header = (repo / "CarpetX/src/driver.hxx").read_text()
    fillpatch = (repo / "CarpetX/src/fillpatch.cxx").read_text()
    openpmd = (repo / "CarpetX/src/io_openpmd.cxx").read_text()
    silo = (repo / "CarpetX/src/io_silo.cxx").read_text()

    # The schema stores source-space constituents, never independently
    # prolonged constituents, plus one accepted boundary snapshot.
    for token in ("ks_source", "old_source", "accepted_consumer"):
        assert token in header
        assert token in driver
        assert f"band_kind::{token}" in openpmd
        assert f"band_kind::{token}" in silo
    for obsolete in ("band_kind::ks_consumer", "band_kind::old_consumer"):
        assert obsolete not in openpmd
        assert obsolete not in silo

    # The boundary snapshot is gathered from the actual tl=0 state at output
    # time, after PostStep, rather than re-created from temporal constituents.
    for backend in (openpmd, silo):
        snapshot = backend.index("SnapshotCoarseFineStateToBand")
        write_accepted = backend.index("band_kind::accepted_consumer", snapshot)
        assert snapshot < write_accepted
        assert "*groupdata.mfab.at(0)" in backend[snapshot:write_accepted]
    assert fillpatch.index("consumer_band.setVal(0.0)") \
        < fillpatch.index("consumer_band.ParallelCopy")
    print("checkpoint wiring: source history + accepted ghost snapshot pass")


if __name__ == "__main__":
    check_rk3_rk4_ordering()
    check_coefficient_values()
    check_production_ordering()
    check_checkpoint_contract()
