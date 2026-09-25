"""MQTT telemetry / command bridge (paho-mqtt >= 2).

Publishes JSON telemetry for SCADA, Node-RED, Grafana/InfluxDB, Home
Assistant or a cloud IoT hub, and accepts *validated* remote setpoints.

Topics (``base`` = e.g. ``agri/site1/greenhouse3``):
  <base>/telemetry        JSON, every control cycle (QoS 0)
  <base>/alarms           JSON per alarm (QoS 1, retained = last alarm)
  <base>/setpoint/set     remote setpoint in degC (QoS 1)
"""
from __future__ import annotations

import json
import threading
from typing import Optional


class MqttBridge:
    """Publishes telemetry and alarms; accepts range-checked remote setpoints."""
    def __init__(self, host: str, base_topic: str, port: int = 1883, username: Optional[str] = None,
                 password: Optional[str] = None, tls: bool = False, setpoint_limits=(0.0, 40.0)):
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError("pip install paho-mqtt") from exc
        self.base = base_topic.rstrip("/")
        self.limits = setpoint_limits
        self._setpoint: Optional[float] = None
        self._lock = threading.Lock()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if username:
            self.client.username_pw_set(username, password)
        if tls:
            self.client.tls_set()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.will_set(f"{self.base}/status", "offline", qos=1, retain=True)
        self.client.connect(host, port)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe(f"{self.base}/setpoint/set", qos=1)
        client.publish(f"{self.base}/status", "online", qos=1, retain=True)

    def _on_message(self, client, userdata, msg):
        try:
            value = float(msg.payload.decode())
        except ValueError:
            return
        lo, hi = self.limits
        if lo <= value <= hi:                      # never trust the network blindly
            with self._lock:
                self._setpoint = value

    @property
    def remote_setpoint(self) -> Optional[float]:
        """Last valid remote setpoint, or None."""
        with self._lock:
            return self._setpoint

    def publish_telemetry(self, payload: dict) -> None:
        self.client.publish(f"{self.base}/telemetry", json.dumps(payload), qos=0)

    def publish_alarm(self, alarm: dict) -> None:
        self.client.publish(f"{self.base}/alarms", json.dumps(alarm), qos=1, retain=True)

    def close(self) -> None:
        self.client.publish(f"{self.base}/status", "offline", qos=1, retain=True)
        self.client.loop_stop()
        self.client.disconnect()
