import numpy as np
import pytest

from agriclimate.control.legacy_fis import DEFAULT_FIS
from agriclimate.fuzzy import MamdaniFIS, MembershipFunction, Rule, Variable, fuzzy_pi_rulebase, load_fis, save_fis
from agriclimate.fuzzy.codegen import build_lut, interp_lut, to_c_header, to_iec_st
from agriclimate.fuzzy.membership import trapmf, trimf


def test_membership_functions():
    assert trimf(5, [0, 5, 10]) == pytest.approx(1.0)
    assert trimf(2.5, [0, 5, 10]) == pytest.approx(0.5)
    assert trimf(11, [0, 5, 10]) == 0.0
    assert trapmf(3, [0, 2, 4, 6]) == pytest.approx(1.0)
    assert trapmf(5, [0, 2, 4, 6]) == pytest.approx(0.5)


def test_single_rule_centroid_is_symmetric_mf_center():
    inp = Variable("x", [0, 10], [MembershipFunction("A", "trimf", [0, 5, 10])])
    out = Variable("y", [0, 10], [MembershipFunction("B", "trimf", [2, 4, 6])])
    fis = MamdaniFIS("t", [inp], [out], [Rule([1], [1])], resolution=2001)
    assert fis(5.0)[0] == pytest.approx(4.0, abs=1e-3)


def test_load_original_fis():
    fis = load_fis(DEFAULT_FIS)
    assert [v.name for v in fis.inputs] == ["sensed_temp", "targeted_temp"]
    assert len(fis.rules) == 17
    # cold greenhouse, warm target -> strong heating, little cooling
    p, n = fis(-30.0, 30.0)
    assert p > 150 and n < 60


def test_legacy_fis_has_coverage_gaps_and_heat_cool_overlap():
    fis = load_fis(DEFAULT_FIS)
    gaps = fis.coverage_gaps(n=41)
    assert max(gaps.values()) > 0.0
    p, n = fis(20.0, 20.0)        # at setpoint both outputs are on -> actuators fight
    assert p > 40 and n > 40


def test_fis_roundtrip(tmp_path):
    fis = fuzzy_pi_rulebase(resolution=201)
    save_fis(fis, tmp_path / "x.fis")
    back = load_fis(tmp_path / "x.fis", resolution=201)
    for x, y in [(0.3, -0.2), (-0.9, 0.9), (0.0, 0.0)]:
        assert back(x, y)[0] == pytest.approx(fis(x, y)[0], abs=1e-9)


def test_fuzzy_pi_surface_is_odd_and_monotonic():
    fis = fuzzy_pi_rulebase(resolution=401)
    assert fis(0.0, 0.0)[0] == pytest.approx(0.0, abs=1e-9)
    assert fis(0.5, 0.2)[0] == pytest.approx(-fis(-0.5, -0.2)[0], abs=1e-6)
    vals = [fis(e, 0.0)[0] for e in np.linspace(-1, 1, 21)]
    assert all(b >= a - 1e-9 for a, b in zip(vals, vals[1:]))


def test_lut_matches_engine_and_codegen_emits_code():
    fis = fuzzy_pi_rulebase(resolution=401)
    lut = build_lut(lambda x, y: fis(x, y)[0], (-1, 1), (-1, 1), 41, 41)
    for x, y in [(0.12, -0.33), (0.8, 0.1), (-0.45, 0.6)]:
        assert interp_lut(*lut, x, y) == pytest.approx(fis(x, y)[0], abs=0.03)
    assert "fuzzy_lut_eval" in to_c_header(*lut)
    st = to_iec_st(*lut)
    assert "FUNCTION_BLOCK" in st and "END_FUNCTION_BLOCK" in st
