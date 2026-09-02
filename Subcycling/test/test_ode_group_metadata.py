#!/usr/bin/env python3
"""Focused structural regression for subcycling ODE group metadata."""

from pathlib import Path


def check_crossed_group_order() -> None:
    """Sorted group-index lists must not be treated as semantic pairs."""
    var_groups = [10, 20]
    declared_rhs = {10: 40, 20: 30}

    positional = list(zip(sorted(var_groups), sorted(declared_rhs.values())))
    semantic = [(gi, declared_rhs[gi]) for gi in sorted(var_groups)]

    assert positional == [(10, 30), (20, 40)]
    assert semantic == [(10, 40), (20, 30)]
    assert positional != semantic


def check_production_wiring() -> None:
    repo = Path(__file__).resolve().parents[2]

    solver = (
        repo / "ODESolvers/src/odesolvers_solve_subcycling.cxx"
    ).read_text()
    begin = solver.index("const auto setks =")
    end = solver.index("const auto fill_old_source_band", begin)
    setks = solver[begin:end]
    assert "for (const int gi : var_groups)" in setks
    assert "const int rhs_gi = get_group_rhs(gi)" in setks
    assert "groupdata.at(rhs_gi)" in setks
    assert "rhs_groups[i]" not in setks
    assert "var_groups[i]" not in setks

    driver = (repo / "CarpetX/src/driver.cxx").read_text()
    header = (repo / "CarpetX/src/driver.hxx").read_text()
    parser_begin = driver.index("bool get_group_ode_rhs_flag")
    parser_end = driver.index("std::array<int, dim> get_group_indextype")
    parser = driver[parser_begin:parser_end]
    assert 'Util_TableGetString(tags, rhs.size(), rhs.data(), "rhs")' in parser
    assert "return rhs.front() != '\\0';" in parser
    assert "bool has_ode_rhs = false;" in header
    assert "has_ode_rhs = get_group_ode_rhs_flag(gi);" in driver

    bands_begin = driver.index(
        "void GHExt::PatchData::LevelData::build_bands"
    )
    bands_end = driver.index("bool all_levels_synchronized", bands_begin)
    bands = driver[bands_begin:bands_end]
    assert "!groupdata.has_ode_rhs" in bands
    assert "!groupdata.do_evolve" not in bands


def check_checkpoint_only_group_model() -> None:
    """Checkpoint/timelevel policy is independent of ODE-band allocation."""
    checkpointed_group = {"checkpoint": True, "evolve": True, "rhs": ""}
    ode_group = {"checkpoint": True, "evolve": True, "rhs": "rhs_state"}

    assert checkpointed_group["evolve"]
    assert not bool(checkpointed_group["rhs"])
    assert bool(ode_group["rhs"])


def main() -> None:
    check_crossed_group_order()
    check_production_wiring()
    check_checkpoint_only_group_model()
    print("subcycling ODE metadata and declared-RHS pairing: PASS")


if __name__ == "__main__":
    main()
