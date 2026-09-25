import math

import pytest

from agriclimate.control import (BroodingSchedule, ControlContext, DayNightSchedule, FuzzyPIController,
                                 PIDController, SafetySupervisor, SplitRangeAllocator, SupervisorConfig)
from agriclimate.plant.facility import ActuatorCommand


def ctx(t, T, sp, dt=10.0, t_out=5.0):
    return ControlContext(t_s=t, dt=dt, temperature=T, setpoint=sp, t_out=t_out)


@pytest.mark.parametrize("ctrl", [FuzzyPIController(), PIDController()])
def test_controllers_push_in_the_right_direction(ctrl):
    ctrl.reset()
    u = [ctrl.update(ctx(10.0 * k, 15.0, 20.0)) for k in range(30)]
    assert u[-1] > 0.0                       # too cold -> heat
    ctrl.reset()
    u = [ctrl.update(ctx(10.0 * k, 25.0, 20.0)) for k in range(30)]
    assert u[-1] < 0.0                       # too warm -> cool


def test_fuzzy_pi_output_is_bounded():
    c = FuzzyPIController()
    for k in range(5000):
        u = c.update(ctx(10.0 * k, -20.0, 30.0))
    assert u == pytest.approx(1.0)


def test_allocator_never_heats_and_cools_together():
    a = SplitRangeAllocator(vent_stage=0.5)
    for d in [-1, -0.7, -0.3, -0.01, 0, 0.01, 0.4, 1]:
        cmd = a.allocate(d, t_air=25, t_out=10)
        assert not (cmd.heater > 0 and cmd.cooler > 0)
    assert a.allocate(-0.3, 25, 10).vent > 0 and a.allocate(-0.3, 25, 10).cooler == 0   # free cooling first
    assert a.allocate(-0.3, 25, 30).vent == 0 and a.allocate(-0.3, 25, 30).cooler > 0   # hot outside


def test_supervisor_sensor_fusion_and_failsafe():
    s = SafetySupervisor(SupervisorConfig())
    v, q, _ = s.validate([20.0, 20.2], 0.0, 10.0)
    assert q == "GOOD" and v == pytest.approx(20.1)
    v, q, _ = s.validate([20.1, math.nan], 10.0, 10.0)
    assert q == "DEGRADED" and v == pytest.approx(20.1)
    s2 = SafetySupervisor(SupervisorConfig(stale_timeout_s=0))
    v, q, al = s2.validate([math.nan, math.nan], 0.0, 10.0)
    assert q == "BAD" and any(a.code == "NO_VALID_SENSOR" for a in al)
    out = s2.apply(ActuatorCommand(0, 1, 1), v, q, 0.0, 10.0, t_out=-5)
    assert out.mode == "FAILSAFE" and out.command.heater > 0 and out.command.cooler == 0


def test_supervisor_limits_interlock_and_hysteresis():
    s = SafetySupervisor(SupervisorConfig(t_low_limit=2, t_high_limit=35, limit_hysteresis=1.0))
    assert s.apply(ActuatorCommand(0, 1, 1), 1.0, "GOOD", 0, 10).mode == "LOW_LIMIT"
    assert s.apply(ActuatorCommand(0, 1, 1), 2.5, "GOOD", 10, 10).mode == "LOW_LIMIT"   # latched
    assert s.apply(ActuatorCommand(0, 0, 0), 3.5, "GOOD", 20, 10).mode == "AUTO"
    out = s.apply(ActuatorCommand(0.6, 0.4, 0), 20, "GOOD", 30, 10)
    assert out.command.cooler == 0.0


def test_supervisor_anti_short_cycle():
    s = SafetySupervisor(SupervisorConfig(max_slew_per_s=0, compressor_min_on_s=100, compressor_min_off_s=200))
    assert s.apply(ActuatorCommand(cooler=1), 20, "GOOD", 0, 10).command.cooler > 0
    assert s.apply(ActuatorCommand(cooler=0), 20, "GOOD", 50, 10).command.cooler > 0     # min-on
    assert s.apply(ActuatorCommand(cooler=0), 20, "GOOD", 120, 10).command.cooler == 0
    assert s.apply(ActuatorCommand(cooler=1), 20, "GOOD", 150, 10).command.cooler == 0   # min-off


def test_schedules():
    dn = DayNightSchedule(day=22, night=16, day_start_h=6, day_end_h=18, ramp_h=2)
    assert dn(3 * 3600) == 16 and dn(12 * 3600) == 22 and dn(7 * 3600) == pytest.approx(19)
    b = BroodingSchedule(start=32, final=21, drop_per_day=0.5)
    assert b(0) == 32 and b(10 * 86400) == 27 and b(60 * 86400) == 21


def test_three_sensors_use_median_and_two_disagreeing_use_mean():
    s = SafetySupervisor(SupervisorConfig())
    v, q, _ = s.validate([20.0, 20.1, 25.0], 0.0, 10.0)      # one faulty of three
    assert v == pytest.approx(20.1) and q == "DEGRADED"
    s2 = SafetySupervisor(SupervisorConfig())
    v, q, al = s2.validate([20.0, 23.0], 0.0, 10.0)           # two disagreeing
    assert v == pytest.approx(21.5) and q == "DEGRADED"
    assert any(a.code == "SENSOR_DISAGREE" for a in al)
