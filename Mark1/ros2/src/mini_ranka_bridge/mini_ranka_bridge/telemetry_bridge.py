#!/usr/bin/env python3
"""Serial-JSON -> ROS2 telemetry bridge for the Mini_Ranka arm.

The firmware (lib/Telemetry) streams newline-delimited JSON over USB serial:

    {"t":12873,"cmd1":1.2000,"meas1":1.1834,"err1":0.0166, ...}

This node reads those lines and republishes every numeric field as its own
``std_msgs/Float64`` topic under a namespace, creating publishers lazily the
first time a key is seen. That makes it fully schema-agnostic: add a signal in
the firmware and a new topic appears here automatically, with no host changes.
PlotJuggler subscribes to the topics (ROS2 Topic Subscriber); ``ros2 bag
record`` handles logging.

It is bidirectional: it also subscribes to ``<ns>/cmd`` (``std_msgs/String``)
and writes each line to the same serial port, so SimpleFOC Commander commands
(PID gains, step toggles) can be driven over ROS2 while tuning. Serial lines
that are not telemetry JSON (boot banners, Commander replies) are echoed on
``<ns>/log`` so the host can read them back.

The firmware <-> bridge contract is JUST the JSON line shape above, so either
side can be swapped independently (see docs/plotjuggler.md).
"""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

import serial  # pyserial


class TelemetryBridge(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_bridge")

        # --- parameters (override via launch / params.yaml / CLI) -----------
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = int(self.declare_parameter("baud", 921600).value)
        ns = str(self.declare_parameter("namespace", "mini_ranka").value)
        self.ns = ns.strip("/")
        self.reconnect_period = float(
            self.declare_parameter("reconnect_period", 2.0).value
        )
        # Commander lines sent once the port is open, e.g. to pick which dynamics
        # feedforward terms are active without reflashing:  ["DN", "DG", "F"].
        # Replayed on every reconnect, since the board usually resets when the
        # port is reopened and would otherwise fall back to its config.h defaults.
        self.startup_commands = [
            str(c) for c in self.declare_parameter("startup_commands", []).value or []
        ]
        # Seconds to wait after opening before sending them — the STM32 VCP drops
        # anything written while the firmware is still in setup().
        self.startup_delay = float(self.declare_parameter("startup_delay", 2.0).value)

        self._pubs: dict[str, "rclpy.publisher.Publisher"] = {}
        self._serial: "serial.Serial | None" = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()

        # Outbound: non-JSON serial lines (banners, Commander replies) -> <ns>/log.
        self._log_pub = self.create_publisher(String, self._topic("log"), 10)
        # Inbound: forward String commands to the firmware's SimpleFOC Commander.
        self.create_subscription(String, self._topic("cmd"), self._on_cmd, 10)

        self.get_logger().info(
            f"telemetry_bridge: {self.port} @ {self.baud}  "
            f"out -> {self._topic('<signal>')}, {self._topic('log')}  "
            f"in <- {self._topic('cmd')}"
        )
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ----------------------------------------------------------------------
    def _topic(self, name: str) -> str:
        """Build a fully-qualified topic name under the configured namespace."""
        return f"/{self.ns}/{name}" if self.ns else f"/{name}"

    def _publisher_for(self, key: str):
        """Return (creating on first use) the Float64 publisher for `key`."""
        pub = self._pubs.get(key)
        if pub is None:
            topic = self._topic(key)
            pub = self.create_publisher(Float64, topic, 10)
            self._pubs[key] = pub
            self.get_logger().info(f"new signal -> {topic}")
        return pub

    def _publish_line(self, line: str) -> None:
        """Parse one JSON line and publish each numeric field; echo the rest."""
        try:
            obj = json.loads(line)
        except (json.JSONDecodeError, ValueError):
            self._log_pub.publish(String(data=line))  # banner / Commander reply
            return
        if not isinstance(obj, dict):
            return
        for key, val in obj.items():
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                continue
            self._publisher_for(str(key)).publish(Float64(data=float(val)))

    def _write_line(self, text: str) -> bool:
        """Write one newline-terminated Commander line to the firmware."""
        ser = self._serial
        if ser is None or not ser.is_open:
            self.get_logger().warning(f"cmd dropped (port not open): {text!r}")
            return False
        try:
            with self._write_lock:
                ser.write((text.rstrip("\r\n") + "\n").encode("utf-8"))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"cmd write failed: {exc}")
            return False
        self.get_logger().info(f"cmd -> {text!r}")
        return True

    def _on_cmd(self, msg: "String") -> None:
        """Forward one command line from ROS2 to the firmware over serial."""
        self._write_line(msg.data)

    def _send_startup_commands(self) -> None:
        """Replay the configured Commander lines after a (re)connect."""
        if not self.startup_commands:
            return
        if self._stop.wait(self.startup_delay):   # bail out if shutting down
            return
        for cmd in self.startup_commands:
            if not self._write_line(cmd):
                return
            self._stop.wait(0.05)   # let the firmware consume each line

    def _run(self) -> None:
        """Reader loop: (re)open the port and pump lines until shutdown."""
        while not self._stop.is_set() and rclpy.ok():
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=1.0)
                self.get_logger().info(f"opened {self.port}")
                # Off-thread so the reader starts pumping immediately instead of
                # blocking for startup_delay while the board boots.
                threading.Thread(
                    target=self._send_startup_commands, daemon=True
                ).start()
            except serial.SerialException as exc:
                self.get_logger().warning(
                    f"cannot open {self.port}: {exc}; retrying in "
                    f"{self.reconnect_period:.1f}s"
                )
                self._stop.wait(self.reconnect_period)
                continue

            try:
                while not self._stop.is_set() and rclpy.ok():
                    raw = self._serial.readline()
                    if not raw:
                        continue  # read timeout, just poll again
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line:
                        self._publish_line(line)
            except serial.SerialException as exc:
                self.get_logger().warning(f"serial error: {exc}; reconnecting")
            finally:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
