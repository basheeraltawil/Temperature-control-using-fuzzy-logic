"""Modbus TCP link to a PLC / remote I/O (pymodbus >= 3).

The PLC owns the physical I/O and the last line of defence: it runs the
generated fuzzy-PI function block as a fallback and switches to it when the
edge computer's heartbeat stops.  The edge computer (this code) adds the
supervisory functions - MPC, AI monitoring, recipes, logging.

Register map (defaults, all 16-bit):
  input registers   0..n-1   temperature sensors, degC x 100 (signed)
                    n        indoor RH, % x 100
                    n+1      outdoor temperature, degC x 100 (signed)
                    n+2      global radiation, W/m2
  holding registers 0        heater command   0..10000 = 0..100.00 %
                    1        cooler command   0..10000
                    2        vent command     0..10000
                    3        heartbeat counter (PLC watchdog: stale > 5 s -> local control)
                    4        mode word: 0 = local (PLC FB), 1 = remote (edge)
"""
from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import Dict, List

from ..plant.facility import ActuatorCommand


@dataclass
class RegisterMap:
    """Where the registers start and how many temperature sensors there are."""
    n_temperature: int = 2
    input_base: int = 0
    holding_base: int = 0
    unit: int = 1


def _signed(v: int) -> int:
    return v - 0x10000 if v >= 0x8000 else v


class ModbusClimateIO:
    """Reads sensors from and writes commands + heartbeat to the PLC."""
    def __init__(self, host: str, port: int = 502, regmap: RegisterMap = RegisterMap(), timeout: float = 1.0):
        try:
            from pymodbus.client import ModbusTcpClient
        except ImportError as exc:
            raise RuntimeError("pip install pymodbus") from exc
        self.client = ModbusTcpClient(host, port=port, timeout=timeout)
        self.map = regmap
        self._heartbeat = 0
        # pymodbus renamed 'slave' to 'device_id' in 3.10
        params = inspect.signature(self.client.read_input_registers).parameters
        self._unit_kw = "device_id" if "device_id" in params else "slave"

    def connect(self) -> bool:
        return bool(self.client.connect())

    def close(self) -> None:
        self.client.close()

    def read(self) -> Dict[str, object]:
        """Read temperatures (degC), RH (%), outdoor temperature (degC) and radiation (W/m2)."""
        m = self.map
        count = m.n_temperature + 3
        rr = self.client.read_input_registers(m.input_base, count=count, **{self._unit_kw: m.unit})
        if rr.isError():
            raise IOError(f"Modbus read failed: {rr}")
        regs = rr.registers
        temps: List[float] = [_signed(r) / 100.0 for r in regs[: m.n_temperature]]
        return {"temperatures": temps, "rh": regs[m.n_temperature] / 100.0,
                "t_out": _signed(regs[m.n_temperature + 1]) / 100.0, "solar": float(regs[m.n_temperature + 2])}

    def write(self, cmd: ActuatorCommand, remote: bool = True) -> None:
        """Write heater/cooler/vent (0..10000), the heartbeat counter and the remote flag."""
        c = cmd.clipped()
        self._heartbeat = (self._heartbeat + 1) % 0x10000
        values = [round(c.heater * 10000), round(c.cooler * 10000), round(c.vent * 10000),
                  self._heartbeat, 1 if remote else 0]
        wr = self.client.write_registers(self.map.holding_base, values, **{self._unit_kw: self.map.unit})
        if wr.isError():
            raise IOError(f"Modbus write failed: {wr}")
