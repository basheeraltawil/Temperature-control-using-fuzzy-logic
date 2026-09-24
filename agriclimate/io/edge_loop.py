"""Real-time edge controller without ROS: PLC over Modbus TCP + MQTT telemetry.

    python -m agriclimate.io.edge_loop --plc 192.168.0.10 --mqtt broker.local \
        --scenario tomato_greenhouse_spring --controller fuzzy_pi

The scenario file supplies the setpoint recipe, allocator and supervisor
limits that were validated in simulation.
"""
from __future__ import annotations

import argparse
import logging
import time

from ..runtime import ClimateRuntime
from ..sim.runner import make_controller
from ..sim.scenario import Scenario
from .modbus_io import ModbusClimateIO, RegisterMap

log = logging.getLogger("agriclimate.edge")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plc", required=True, help="PLC / remote-I/O IP address")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=1)
    ap.add_argument("--sensors", type=int, default=2)
    ap.add_argument("--mqtt", help="MQTT broker host (optional)")
    ap.add_argument("--topic", default="agri/site1/zone1")
    ap.add_argument("--scenario", default="tomato_greenhouse_spring")
    ap.add_argument("--controller", default="fuzzy_pi", choices=["fuzzy_pi", "pid"])
    ap.add_argument("--period", type=float, default=10.0, help="control period, s")
    ap.add_argument("--time-scale", type=float, default=1.0,
                    help="only for accelerated tests against plc_simulator (must match its --time-scale)")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    sc = Scenario.load(args.scenario)
    ctrl, _ = make_controller(args.controller, sc)
    rt = ClimateRuntime(ctrl, sc.allocator(), sc.supervisor_config(), supervised=True)
    io = ModbusClimateIO(args.plc, args.port, RegisterMap(n_temperature=args.sensors, unit=args.unit))
    if not io.connect():
        raise SystemExit(f"cannot connect to PLC at {args.plc}:{args.port}")
    mqtt = None
    if args.mqtt:
        from .mqtt_bridge import MqttBridge

        mqtt = MqttBridge(args.mqtt, args.topic)
    lt = time.localtime()
    recipe_offset = lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec   # recipe day 0 = today 00:00 local
    t0 = time.monotonic()
    next_t = t0
    try:
        while True:
            now = time.monotonic()
            t = (now - t0) * args.time_scale
            try:
                d = io.read()
                readings = d["temperatures"]
            except IOError as exc:            # field-bus fault -> supervisor sees invalid sensors
                log.warning("%s", exc)
                d, readings = {"rh": 60.0, "t_out": 5.0, "solar": 0.0}, [float("nan")] * args.sensors
            sp = mqtt.remote_setpoint if mqtt and mqtt.remote_setpoint is not None else sc.setpoint(
                recipe_offset + t)
            out = rt.step(t, args.period * args.time_scale, readings, sp, d["rh"], d["t_out"], solar=d["solar"])
            try:
                io.write(out.command)
            except IOError as exc:
                log.error("%s (PLC watchdog will take over)", exc)
            for a in out.alarms:
                log.warning("%s %s", a.code, a.message)
                if mqtt:
                    mqtt.publish_alarm(a.as_dict())
            if mqtt:
                mqtt.publish_telemetry({"t": time.time(), "setpoint": sp, "temperature": out.temperature,
                                        "quality": out.quality, "mode": out.mode, **out.command.__dict__})
            next_t += args.period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        io.close()
        if mqtt:
            mqtt.close()


if __name__ == "__main__":
    main()
