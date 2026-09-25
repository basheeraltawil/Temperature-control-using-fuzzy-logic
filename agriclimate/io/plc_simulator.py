"""Virtual PLC: serves the digital twin over Modbus TCP (virtual commissioning).

Run it, then point ``agriclimate.io.edge_loop`` (or the ROS 2 Modbus bridge,
or a real SCADA) at it to test the complete field chain - register map,
scaling, watchdog, fail-safe - before touching the real greenhouse:

    python -m agriclimate.io.plc_simulator --port 5020 --time-scale 30
    python -m agriclimate.io.edge_loop --plc 127.0.0.1 --port 5020 --period 1 --time-scale 30

Tested with pymodbus 3.6 - 3.9 (3.10+ is migrating its datastore API towards v4).

The simulated PLC also implements the heartbeat watchdog: when the edge
computer stops updating the heartbeat for ``watchdog_s``, it falls back to
its own local fuzzy-PI function block (here: the Python twin of the
generated IEC 61131-3 code).
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import time

from ..control.base import ControlContext
from ..control.fuzzy_pi import FuzzyPIController
from ..plant.facility import ActuatorCommand, Facility
from ..plant.sensors import Sensor
from ..sim.scenario import Scenario

log = logging.getLogger("agriclimate.plc_sim")


def _u16(v: float) -> int:
    return int(round(v)) & 0xFFFF


def _datastore(n_input: int, n_holding: int):
    import pymodbus.datastore as ds

    dev_cls = getattr(ds, "ModbusDeviceContext", None) or getattr(ds, "ModbusSlaveContext")
    # block address 1 == protocol address 0 (the device context adds the offset)
    dev = dev_cls(ir=ds.ModbusSequentialDataBlock(1, [0] * (n_input + 1)),
                  hr=ds.ModbusSequentialDataBlock(1, [0] * (n_holding + 1)))
    try:
        ctx = ds.ModbusServerContext(devices=dev, single=True)
    except TypeError:                                    # pymodbus < 3.10
        ctx = ds.ModbusServerContext(slaves=dev, single=True)
    return dev, ctx


class VirtualPLC:
    """Digital twin behind a Modbus TCP server, with heartbeat watchdog and local fuzzy-PI."""
    def __init__(self, scenario: Scenario, n_sensors: int = 2, period: float = 1.0, time_scale: float = 1.0,
                 watchdog_s: float = 5.0):
        self.sc = scenario
        self.plant = Facility(scenario.facility, scenario.initial.get("t_air", 18.0), scenario.initial.get("rh", 70.0))
        self.weather = scenario.weather()
        self.sensors = [Sensor(name=f"T{i + 1}", seed=i) for i in range(n_sensors)]
        self.n = n_sensors
        self.period, self.scale, self.watchdog_s = period, time_scale, watchdog_s
        self.dev, self.context = _datastore(n_sensors + 3, 5)
        self.local = FuzzyPIController()                 # PLC-resident fallback controller
        self.allocator = scenario.allocator()
        lt = time.localtime()     # same recipe/weather clock as the edge loop: day 0 = today 00:00
        self.t = float(lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec)
        self._last_hb, self._hb_age = None, 0.0
        self.mode = "LOCAL"

    def _step(self) -> None:
        dt = self.period * self.scale
        w = self.weather.sample(self.t)
        temps = [s.read(self.plant.state.t_air, self.t, dt) for s in self.sensors]
        regs = [_u16(v * 100) for v in temps] + [_u16(self.plant.rh * 100), _u16(w.t_out * 100), _u16(w.solar)]
        self.dev.setValues(4, 0, regs)
        heater, cooler, vent, hb, remote = self.dev.getValues(3, 0, 5)
        if hb != self._last_hb:
            self._last_hb, self._hb_age = hb, 0.0
        else:
            self._hb_age += self.period
        if remote and self._hb_age < self.watchdog_s:
            if self.mode != "REMOTE":
                log.info("edge controller online -> REMOTE")
            self.mode = "REMOTE"
            cmd = ActuatorCommand(heater / 10000, cooler / 10000, vent / 10000)
        else:
            if self.mode == "REMOTE":
                log.warning("heartbeat lost -> LOCAL fuzzy-PI function block")
                self.local.reset()
            self.mode = "LOCAL"
            sp = self.sc.setpoint(self.t)
            u = self.local.update(ControlContext(self.t, dt, sum(temps) / len(temps), sp, t_out=w.t_out))
            cmd = self.allocator.allocate(u, temps[0], w.t_out, self.plant.rh)
        self.plant.step(dt, cmd, w)
        self.t += dt

    async def _loop(self) -> None:
        while True:
            self._step()
            if int(self.t) % 600 < self.period * self.scale:
                log.info("clock=%05.2fh T=%.2f degC setpoint=%.1f mode=%s", (self.t / 3600) % 24,
                         self.plant.state.t_air, self.sc.setpoint(self.t), self.mode)
            await asyncio.sleep(self.period)

    async def serve(self, host: str = "0.0.0.0", port: int = 5020) -> None:
        from pymodbus.server import StartAsyncTcpServer

        task = asyncio.create_task(self._loop())
        try:
            await StartAsyncTcpServer(context=self.context, address=(host, port))
        finally:
            task.cancel()


def main(argv=None) -> None:
    """Start the virtual PLC server."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5020)
    ap.add_argument("--scenario", default="tomato_greenhouse_spring")
    ap.add_argument("--time-scale", type=float, default=1.0, help="simulated seconds per real second")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    plc = VirtualPLC(Scenario.load(args.scenario), time_scale=args.time_scale)
    asyncio.run(plc.serve(args.host, args.port))


if __name__ == "__main__":
    main()
