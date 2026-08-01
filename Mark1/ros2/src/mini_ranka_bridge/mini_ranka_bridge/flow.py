#!/usr/bin/env python3
"""Live diagram of the arm's signal path, from the trajectory down to RViz.

A component is lit when data is actually moving through it, not when its process
happens to exist. ``rqt_graph`` answers "what is running"; the question that
matters mid-experiment is "where did the data stop", and those are different
questions — a bridge whose serial port dropped is still a node in the graph.

So every box is driven by a measured rate or a measured value:

    telemetry_bridge      Hz on <ns>/t          (not: node is alive)
    J1..J3                Hz on meas<n>, colour ramped by |err<n>|
    Trajectory            x/y/z actually changing (static => manual T target)
    Cartesian FK          mx/my/mz non-zero     (C parks them at 0 when off)
    Dyn FF                <ns>/log state + whether ff<n> is really non-zero
    /joint_states, /tf    measured publish rate

Firmware feature state has no telemetry channel — ``homed``, ``dyn_terms``,
``cart_meas_enabled`` and friends are only ever visible in the Commander's text
replies. Those are parsed out of ``<ns>/log`` as they scroll past, which is why
the panel starts with them unknown and fills in as you drive the arm. Pressing
'd' sends a bare ``D``, the one Commander command that reports without toggling.

Topics are discovered, never assumed: telemetry_bridge creates its publishers
lazily per JSON key, and the pid_tuner firmware emits a completely different set
(target/meas/err/vel) from main (cmd<n>/meas<n>/q<n>/...). The panel subscribes
to whatever Float64 topics show up under the namespace and names the profile it
recognises.

Usage:
    ros2 run mini_ranka_bridge flow
    ros2 run mini_ranka_bridge flow --ros-args -p namespace:=mini_ranka
"""

import glob
import re
import signal
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_rosout_default
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from python_qt_binding.QtCore import Qt, QPointF, QRectF, QTimer
from python_qt_binding.QtGui import (QBrush, QColor, QFont, QFontMetricsF,
                                     QPainter, QPainterPath, QPen)
from python_qt_binding.QtWidgets import QApplication, QWidget

# --- palette ---------------------------------------------------------------
# Near-monochrome on purpose. The panel has no window background of its own —
# the blocks float directly over the RViz viewport — so colour has to carry
# meaning rather than decorate: cool white is live, steel is idle, and the only
# other hue is the red that tracking error ramps into.
INK = QColor("#eaf4f8")          # live
INK_SOFT = QColor("#9fb6c2")     # live, secondary text
DIM = QColor("#3f4b55")          # idle stroke
DIM_TEXT = QColor("#61707b")     # idle text
ALERT = QColor("#ff6b5e")        # error, and the far end of the error ramp
# Blocks carry their own faint smoked-glass fill so small text stays legible
# over a moving 3D scene without the panel needing a background.
GLASS = QColor(9, 14, 19, 170)
GLASS_IDLE = QColor(9, 14, 19, 105)


def mix(a: QColor, b: QColor, t: float) -> QColor:
    """Blend two colours; the error ramp is INK -> ALERT rather than a hue set."""
    t = max(0.0, min(1.0, t))
    return QColor(round(a.red() + (b.red() - a.red()) * t),
                  round(a.green() + (b.green() - a.green()) * t),
                  round(a.blue() + (b.blue() - a.blue()) * t))

# Design-space size. Everything is laid out in these units and scaled to fit the
# window, so the panel survives being resized or moved to the other monitor.
DW, DH = 664.0, 626.0

STALE = 1.0        # s without a message before a signal counts as dead
RATE_WINDOW = 32   # samples kept per topic for the Hz estimate

# Telemetry key sets, used only to name the profile on the wire.
MAIN_KEYS = {"cmd1", "meas1", "err1", "q1", "ff1", "mx"}
TUNER_KEYS = {"target", "meas", "err", "vel"}

TERM_RE = re.compile(
    r"dynamics terms:\s*gravity=(\w+)\s+inertia=(\w+)\s+"
    r"centrifugal=(\w+)\s+coriolis=(\w+)")
FF_RE = re.compile(r"dynamics feedforward:\s*(\w+)")
CART_RE = re.compile(r"cartesian measurement:\s*(\w+)")
PORT_RE = re.compile(r"opened (/dev/\S+)")


class Rate:
    """Rolling arrival-rate estimate for one topic."""

    def __init__(self) -> None:
        self.stamps = deque(maxlen=RATE_WINDOW)
        self.value = 0.0

    def tick(self, value: float = 0.0) -> None:
        self.stamps.append(time.monotonic())
        self.value = value

    def hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        now = time.monotonic()
        if now - self.stamps[-1] > STALE:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / span if span > 0 else 0.0

    def fresh(self) -> bool:
        return bool(self.stamps) and time.monotonic() - self.stamps[-1] <= STALE


class FlowMonitor(Node):
    """Watches the graph and the wire; owns all ROS state behind one lock."""

    def __init__(self) -> None:
        super().__init__("flow_monitor")

        ns = str(self.declare_parameter("namespace", "mini_ranka").value).strip("/")
        self.ns = ns
        self.prefix = f"/{ns}/" if ns else "/"
        self.max_topics = int(self.declare_parameter("max_topics", 48).value)

        self._lock = threading.Lock()
        self._rates: dict[str, Rate] = {}          # bare signal name -> Rate
        self._subs: dict[str, object] = {}
        self._nodes: set[str] = set()
        self._traj_hist = deque(maxlen=100)        # (x, y, z) samples
        self._ports: list[str] = []
        self._serial_port = None                   # from rosout "opened ..."
        self._signal_count = 0

        # Firmware state, only knowable from Commander replies. None = not yet seen.
        self.fw = {"terms": None, "ff": None, "cart": None,
                   "homed": False, "traj": None, "direct": None}

        self.js_rate = Rate()
        self.tf_rate = Rate()

        self.create_subscription(String, self.prefix + "log", self._on_log, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.create_subscription(
            Log, "/rosout", self._on_rosout, qos_profile_rosout_default)

        self.cmd_pub = self.create_publisher(String, self.prefix + "cmd", 10)

        # Discovery runs on the node's own executor thread, so subscriptions are
        # only ever created from the thread that spins them.
        self.create_timer(1.0, self._discover)
        self._discover()

    # ------------------------------------------------------------------
    def _discover(self) -> None:
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

        new = []
        for topic, types in self.get_topic_names_and_types():
            if not topic.startswith(self.prefix) or "std_msgs/msg/Float64" not in types:
                continue
            key = topic[len(self.prefix):]
            if "/" in key or key in self._subs:
                continue
            if len(self._subs) >= self.max_topics:
                break
            new.append((topic, key))

        for topic, key in new:
            rate = Rate()
            self._subs[key] = self.create_subscription(
                Float64, topic, self._on_signal(key, rate), 10)
            with self._lock:
                self._rates[key] = rate

        with self._lock:
            self._nodes = names
            self._ports = ports
            self._signal_count = len(self._subs)

    def _on_signal(self, key: str, rate: Rate):
        def cb(msg: Float64) -> None:
            with self._lock:
                rate.tick(msg.data)
                if key in ("x", "y", "z"):
                    self._traj_hist.append((key, msg.data))
        return cb

    def _on_js(self, _msg: JointState) -> None:
        with self._lock:
            self.js_rate.tick()

    def _on_tf(self, _msg: TFMessage) -> None:
        with self._lock:
            self.tf_rate.tick()

    def _on_log(self, msg: String) -> None:
        line = msg.data
        with self._lock:
            if (m := TERM_RE.search(line)):
                self.fw["terms"] = tuple(g == "on" for g in m.groups())
            if (m := FF_RE.search(line)):
                self.fw["ff"] = m.group(1) == "on"
            if (m := CART_RE.search(line)):
                self.fw["cart"] = m.group(1) == "on"
            if "homed offsets" in line:
                self.fw["homed"] = True
            if "Trajectory resumed" in line:
                self.fw["traj"] = True
            elif "Manual target" in line:
                self.fw["traj"] = False

    def _on_rosout(self, msg: Log) -> None:
        with self._lock:
            if (m := PORT_RE.search(msg.msg)):
                self._serial_port = m.group(1)
            elif "cannot open" in msg.msg:
                self._serial_port = None
            elif "using the firmware's joint angles" in msg.msg:
                self.fw["direct"] = True

    # ------------------------------------------------------------------
    def probe(self) -> None:
        """Send a bare 'D'. doDynTerms treats it as report-only; DG/DI/DC/DK
        are XOR toggles, so nothing else may ever be sent from here."""
        self.cmd_pub.publish(String(data="D"))

    def snapshot(self) -> dict:
        """Plain-data view of everything the widget draws, taken under the lock."""
        with self._lock:
            rates = {k: (r.hz(), r.value, r.fresh()) for k, r in self._rates.items()}
            keys = set(rates)
            traj_vals = {}
            for axis, val in self._traj_hist:
                traj_vals.setdefault(axis, set()).add(round(val, 5))
            snap = {
                "rates": rates,
                "nodes": set(self._nodes),
                "ports": list(self._ports),
                "serial_port": self._serial_port,
                "signals": self._signal_count,
                "js_hz": self.js_rate.hz(),
                "tf_hz": self.tf_rate.hz(),
                "fw": dict(self.fw),
                # Moving if any axis took more than one distinct value recently.
                "traj_moving": any(len(v) > 1 for v in traj_vals.values()),
                "profile": ("main" if keys & MAIN_KEYS else
                            "pid_tuner" if keys & TUNER_KEYS else None),
            }
        return snap


# --- drawing ---------------------------------------------------------------

class FlowWidget(QWidget):
    def __init__(self, monitor: FlowMonitor) -> None:
        super().__init__()
        self.monitor = monitor
        self.snap = monitor.snapshot()
        self.phase = 0.0
        self.setWindowTitle("mini_ranka · flow")
        self.setMinimumSize(420, 380)
        self.resize(int(DW), int(DH))
        # No window background: Qt clears to transparent every frame, so only
        # the blocks and rules are drawn and RViz shows through between them.
        # Hyprland has to be told to skip its own chrome too — see
        # Mark1/tools/hypr/mini-ranka.conf (border_size 0, no_blur, no_shadow),
        # otherwise a blurred rounded rectangle reappears around the "floating"
        # blocks.
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

        self.mono = QFont("JetBrains Mono")
        self.mono.setStyleHint(QFont.StyleHint.Monospace)
        if not self.mono.exactMatch():
            self.mono = QFont("DejaVu Sans Mono")
            self.mono.setStyleHint(QFont.StyleHint.Monospace)

        timer = QTimer(self)
        timer.timeout.connect(self._tick)
        timer.start(100)

    def _tick(self) -> None:
        self.snap = self.monitor.snapshot()
        self.phase -= 1.4          # negative => dashes travel along the arrow
        self.update()

    def keyPressEvent(self, event) -> None:
        key = event.key()
        if key in (Qt.Key.Key_Q, Qt.Key.Key_Escape):
            self.close()
        elif key == Qt.Key.Key_D:
            self.monitor.probe()

    # -- primitives --------------------------------------------------------
    def _font(self, size: float, bold: bool = False) -> QFont:
        f = QFont(self.mono)
        f.setPointSizeF(size)
        f.setWeight(QFont.Weight.Bold if bold else QFont.Weight.Normal)
        return f

    def _text(self, p: QPainter, rect: QRectF, text: str, color: QColor,
              size: float = 8.0, bold: bool = False,
              align=Qt.AlignmentFlag.AlignCenter) -> None:
        p.setFont(self._font(size, bold))
        p.setPen(QPen(color))
        p.drawText(rect, int(align | Qt.AlignmentFlag.AlignVCenter), text)

    def _elide(self, text: str, size: float, bold: bool, width: float) -> str:
        return QFontMetricsF(self._font(size, bold)).elidedText(
            text, Qt.TextElideMode.ElideMiddle, width)

    def _ticks(self, p: QPainter, rect: QRectF, accent: QColor) -> None:
        """HUD corner brackets. Only live blocks get them, so "is this carrying
        data" survives being read at a glance in near-monochrome."""
        n = 6.0
        p.setPen(QPen(accent, 1.2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        for cx, cy, sx, sy in ((rect.left(), rect.top(), 1, 1),
                               (rect.right(), rect.top(), -1, 1),
                               (rect.left(), rect.bottom(), 1, -1),
                               (rect.right(), rect.bottom(), -1, -1)):
            path = QPainterPath(QPointF(cx + sx * n, cy))
            path.lineTo(QPointF(cx, cy))
            path.lineTo(QPointF(cx, cy + sy * n))
            p.drawPath(path)

    def _panel(self, p: QPainter, rect: QRectF, live: bool,
               accent: QColor = INK) -> None:
        """Frame only: smoked glass, hairline edge, brackets when live."""
        path = QPainterPath()
        path.addRoundedRect(rect, 3, 3)
        if live:
            halo = QColor(accent)
            halo.setAlpha(22)
            p.setPen(QPen(halo, 3.5))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawPath(path)
            p.setBrush(QBrush(GLASS))
            p.setPen(QPen(accent, 1.0))
        else:
            p.setBrush(QBrush(GLASS_IDLE))
            p.setPen(QPen(DIM, 1.0))
        p.drawPath(path)
        if live:
            self._ticks(p, rect.adjusted(1.5, 1.5, -1.5, -1.5), accent)

    def _box(self, p: QPainter, rect: QRectF, title: str, sub: str = "",
             live: bool = False, accent: QColor = INK, note: str = "") -> None:
        """One component: title, optional second line."""
        self._panel(p, rect, live, accent)
        body = INK if live else DIM_TEXT
        muted = INK_SOFT if live else DIM_TEXT
        pad = rect.width() - 16
        if sub or note:
            self._text(p, QRectF(rect.x(), rect.y() + 3, rect.width(),
                                 rect.height() * 0.52),
                       self._elide(title, 8.2, True, pad), body, 8.2, True)
            line = QRectF(rect.x(), rect.y() + rect.height() * 0.5,
                          rect.width(), rect.height() * 0.45)
            self._text(p, line, self._elide(sub or note, 7.0, False, pad),
                       accent if (live and note and not sub) else muted, 7.0)
        else:
            self._text(p, rect, self._elide(title, 8.2, True, pad), body,
                       8.2, True)

    def _arrow(self, p: QPainter, pts: list, flowing: bool,
               accent: QColor = INK, head: bool = True) -> None:
        """Polyline with an arrowhead. Dashes animate only while data moves."""
        color = accent if flowing else DIM
        pen = QPen(color, 1.2 if flowing else 0.9)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        if flowing:
            pen.setStyle(Qt.PenStyle.CustomDashLine)
            pen.setDashPattern([4, 3])
            pen.setDashOffset(self.phase)
        else:
            pen.setStyle(Qt.PenStyle.DotLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)

        path = QPainterPath(QPointF(*pts[0]))
        for pt in pts[1:]:
            path.lineTo(QPointF(*pt))
        p.drawPath(path)

        if not head:
            return
        (x0, y0), (x1, y1) = pts[-2], pts[-1]
        dx, dy = x1 - x0, y1 - y0
        length = max((dx * dx + dy * dy) ** 0.5, 1e-6)
        ux, uy = dx / length, dy / length
        size = 5.0
        tip = QPointF(x1, y1)
        left = QPointF(x1 - ux * size - uy * size * 0.55,
                       y1 - uy * size + ux * size * 0.55)
        right = QPointF(x1 - ux * size + uy * size * 0.55,
                        y1 - uy * size - ux * size * 0.55)
        headp = QPainterPath(tip)
        headp.lineTo(left)
        headp.lineTo(right)
        headp.closeSubpath()
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(color))
        p.drawPath(headp)

    def _chip(self, p: QPainter, rect: QRectF, label: str, on: bool,
              accent: QColor = INK) -> None:
        p.setPen(QPen(accent if on else DIM, 1.0))
        p.setBrush(QBrush(accent if on else Qt.GlobalColor.transparent))
        p.drawRect(rect)
        self._text(p, rect, label, QColor("#0b1116") if on else DIM_TEXT,
                   6.4, True)

    def _section(self, p: QPainter, y: float, label: str) -> None:
        """Band heading. A letter-spaced label and a hairline rule instead of a
        filled box — the bands used to be rectangles, which is exactly the
        window-shaped background this panel is meant not to have."""
        text = " ".join(label)
        p.setFont(self._font(6.4, True))
        w = QFontMetricsF(self._font(6.4, True)).horizontalAdvance(text)
        p.setPen(QPen(DIM_TEXT))
        p.drawText(QRectF(24, y, w + 8, 12),
                   int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                   text)
        pen = QPen(DIM, 0.8)
        p.setPen(pen)
        p.drawLine(QPointF(24 + w + 12, y + 6), QPointF(DW - 24, y + 6))

    # -- the diagram -------------------------------------------------------
    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        # Deliberately no fillRect: WA_TranslucentBackground already cleared the
        # frame to transparent, and painting a background here would put back
        # the rectangle the blocks are supposed to float on.

        scale = min(self.width() / DW, self.height() / DH)
        p.translate((self.width() - DW * scale) / 2.0,
                    (self.height() - DH * scale) / 2.0)
        p.scale(scale, scale)

        s = self.snap
        rates = s["rates"]

        def hz(key: str) -> float:
            return rates.get(key, (0.0, 0.0, False))[0]

        def val(key: str) -> float:
            return rates.get(key, (0.0, 0.0, False))[1]

        def live(key: str) -> bool:
            return rates.get(key, (0.0, 0.0, False))[2]

        def node(*fragments: str) -> bool:
            return any(f in n for f in fragments for n in s["nodes"])

        telemetry_up = node("telemetry_bridge")
        mock_up = node("mock_arm")
        # Data actually arriving is what makes the arm "live" — not whether a
        # /dev node happens to be visible from this process.
        wire = live("t") or live("meas1") or live("meas")

        self._header(p, s, wire, telemetry_up, mock_up)
        self._firmware_band(p, s, hz, val, live, wire)
        self._ros_band(p, s, hz, live, node, telemetry_up, mock_up)
        self._legend(p)
        p.end()

    # -- header ------------------------------------------------------------
    def _header(self, p: QPainter, s: dict, wire: bool, telemetry_up: bool,
                mock_up: bool) -> None:
        self._text(p, QRectF(24, 4, 260, 18), "M I N I _ R A N K A", INK,
                   10.0, True, Qt.AlignmentFlag.AlignLeft)

        port = s["serial_port"] or (s["ports"][0] if s["ports"] else "")
        if wire and telemetry_up:
            label, color = f"LIVE  {port}".rstrip(), INK
        elif wire:
            label, color = "STREAMING", INK
        elif telemetry_up:
            label, color = "WAITING FOR ARM", ALERT
        elif mock_up:
            label, color = "MOCK", INK_SOFT
        else:
            label, color = "IDLE", DIM_TEXT
        dot = QRectF(DW - 210, 11, 6, 6)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(color))
        p.drawEllipse(dot)
        self._text(p, QRectF(DW - 196, 4, 172, 18), label, color, 8.0, True,
                   Qt.AlignmentFlag.AlignLeft)
        # Hairline under the header, the one piece of full-width structure left
        # now that the band rectangles are gone.
        p.setPen(QPen(DIM, 0.8))
        p.drawLine(QPointF(24, 28), QPointF(DW - 24, 28))

        # A plugged-in board with no bridge means the session came up in mock and
        # cannot switch by itself — twin.launch.py picks one source at launch.
        if s["ports"] and not telemetry_up:
            self._text(p, QRectF(24, 30, DW - 48, 12),
                       f"{s['ports'][0]} present but no telemetry_bridge — "
                       "SUPER+SHIFT+A then SUPER+A to restart live",
                       ALERT, 6.6, False, Qt.AlignmentFlag.AlignLeft)
        elif s["profile"] == "pid_tuner":
            self._text(p, QRectF(24, 30, DW - 48, 12),
                       "pid_tuner firmware on the wire — joint boxes below are "
                       "the main-firmware layout", ALERT, 6.6, False,
                       Qt.AlignmentFlag.AlignLeft)

    # -- firmware ----------------------------------------------------------
    def _firmware_band(self, p: QPainter, s: dict, hz, val, live,
                       wire: bool) -> None:
        self._section(p, 44, "FIRMWARE")
        fw = s["fw"]

        # Row 1: the setpoint path.
        moving = s["traj_moving"] and wire
        traj_on = fw["traj"] is not False and moving
        self._box(p, QRectF(24, 64, 100, 40), "Trajectory",
                  "running" if traj_on else
                  "manual T" if fw["traj"] is False else "--",
                  traj_on, INK)
        self._box(p, QRectF(156, 64, 64, 40), "IK", "", wire, INK)
        self._box(p, QRectF(252, 64, 84, 40), "limits", "+/-2.0", wire, INK)
        self._arrow(p, [(124, 84), (156, 84)], traj_on)
        self._arrow(p, [(220, 84), (252, 84)], wire)

        cart_on = (abs(val("mx")) + abs(val("my")) + abs(val("mz"))) > 1e-9
        self._box(p, QRectF(396, 64, 122, 40), "Cartesian FK",
                  f"{val('mx'):.2f} {val('my'):.2f} {val('mz'):.2f}"
                  if cart_on else "C to enable", cart_on, INK_SOFT)
        self._box(p, QRectF(530, 64, 106, 40), "homing",
                  "captured" if fw["homed"] else "config.h",
                  fw["homed"], INK)

        # Row 2: feedforward and the move() call it adds into.
        # The terms can all report "on" and still do nothing: Kt is 0 in
        # config.h and torque_to_voltage() returns 0 for kt <= 0, so the whole
        # path is a no-op until the current sensors land. Say so, rather than
        # drawing a lit arrow into move() that carries nothing.
        ff_real = (abs(val("ff1")) + abs(val("ff2")) + abs(val("ff3"))) > 1e-6
        ff_on = bool(fw["ff"]) or ff_real
        ff_accent = INK if ff_real else ALERT
        ff_rect = QRectF(24, 118, 200, 52)
        self._panel(p, ff_rect, ff_on, ff_accent)
        self._text(p, QRectF(34, 122, 96, 18), "Dyn FF",
                   INK if ff_on else DIM_TEXT, 8.2, True,
                   Qt.AlignmentFlag.AlignLeft)
        # Term chips live inside the box: they qualify this node, and putting
        # them outside means an arrow eventually crosses them.
        terms = fw["terms"] or (False, False, False, False)
        for i, letter in enumerate("GICK"):
            self._chip(p, QRectF(136 + i * 21, 124, 17, 13), letter,
                       fw["terms"] is not None and bool(terms[i]), ff_accent)
        if fw["terms"] is None:
            note, note_color = "terms unknown -- press d", DIM_TEXT
        elif ff_real:
            note = (f"ff {val('ff1'):+.2f} {val('ff2'):+.2f} "
                    f"{val('ff3'):+.2f} V")
            note_color = INK
        elif ff_on:
            note, note_color = "on, but Kt=0 in config.h -> 0 V", ALERT
        else:
            note, note_color = "off", DIM_TEXT
        self._text(p, QRectF(34, 146, 182, 16), note, note_color, 7.0, False,
                   Qt.AlignmentFlag.AlignLeft)

        self._box(p, QRectF(252, 118, 84, 52), "move()", "angle", wire, INK)
        self._arrow(p, [(294, 104), (294, 118)], wire)
        self._arrow(p, [(224, 144), (252, 144)], ff_real,
                    INK if ff_real else DIM)

        self._box(p, QRectF(396, 118, 240, 52), "AS5600 x3",
                  "I2C1 / I2C3 / I2C4  --  sensor frame",
                  live("meas1") or live("meas2") or live("meas3"), INK_SOFT)

        # Row 3: the joints, fed from move() by a bus so the three arrows do not
        # each have to find their own way across the band.
        self._arrow(p, [(294, 170), (294, 184)], wire, head=False)
        self._arrow(p, [(74, 184), (306, 184)], wire, head=False)
        for i in range(3):
            n = i + 1
            x = 24 + i * 116
            cx = x + 50
            err = val(f"err{n}")
            joint_live = live(f"meas{n}")
            mag = abs(err)
            # Continuous ramp rather than three named colours: same reading at a
            # glance, one fewer hue in the palette.
            accent = mix(INK, ALERT, mag / 0.25)
            self._arrow(p, [(cx, 184), (cx, 196)], joint_live,
                        accent if joint_live else DIM)
            self._box(p, QRectF(x, 196, 100, 44), f"J{n}",
                      f"err {err:+.3f}" if joint_live else "no data",
                      joint_live, accent)
            if joint_live:
                # Error bar, full width at 0.25 rad.
                w = min(mag / 0.25, 1.0) * 88
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(accent))
                p.drawRoundedRect(QRectF(x + 6, 234, w, 2.4), 1.2, 1.2)

        # Encoders read the joints, back up into the loop.
        self._arrow(p, [(356, 218), (450, 218), (450, 170)],
                    live("meas1"), INK_SOFT)

        # Row 4: the wire out to the host.
        self._box(p, QRectF(24, 252, 150, 36), "USB serial",
                  f"921600 -- {hz('t') or hz('meas'):.0f} Hz JSON" if wire
                  else "no telemetry", wire, INK)
        self._arrow(p, [(99, 288), (99, 326)], wire)

    # -- ros2 --------------------------------------------------------------
    # Three columns: producers (24..174), topics (206..344), consumers
    # (376..546 — wide enough for "robot_state_publisher" without eliding).
    # /tf and /tool_path are drawn as labelled arrows rather than boxes — they
    # are wires, not components, and boxing them forced crossings.
    def _ros_band(self, p: QPainter, s: dict, hz, live, node,
                  telemetry_up: bool, mock_up: bool) -> None:
        self._section(p, 304, "ROS 2")
        fw = s["fw"]
        ns = self.monitor.prefix
        wire_hz = hz("t") or hz("meas")
        flowing = wire_hz > 0

        self._box(p, QRectF(24, 326, 150, 40), "telemetry_bridge",
                  f"{wire_hz:.0f} Hz  ·  {s['signals']} sig" if flowing else
                  (s["serial_port"] or "port closed"),
                  telemetry_up and flowing, INK)
        self._box(p, QRectF(206, 332, 138, 26), f"{ns}*", "", flowing, INK)
        self._box(p, QRectF(206, 372, 138, 26), f"{ns}log", "", telemetry_up,
                  INK)
        self._box(p, QRectF(206, 414, 138, 26), f"{ns}cmd", "",
                  node("tune_console"), INK_SOFT)

        pj = node("plotjuggler", "plot_juggler")
        self._box(p, QRectF(376, 326, 170, 40), "plotjuggler",
                  "streaming" if (pj and flowing) else
                  "no data" if pj else "", pj and flowing, INK_SOFT)
        self._box(p, QRectF(376, 366, 170, 34), "feedback", "",
                  node("feedback_console"), INK)
        self._box(p, QRectF(376, 408, 170, 34), "tune_console", "",
                  node("tune_console"), INK_SOFT)

        self._arrow(p, [(174, 345), (206, 345)], flowing)
        self._arrow(p, [(344, 345), (376, 345)], pj and flowing, INK_SOFT)
        self._arrow(p, [(174, 352), (186, 352), (186, 385), (206, 385)],
                    telemetry_up, INK)
        self._arrow(p, [(344, 385), (376, 383)], node("feedback_console"), INK)
        self._arrow(p, [(376, 427), (344, 427)], node("tune_console"), INK_SOFT)
        self._arrow(p, [(206, 427), (196, 427), (196, 360), (174, 360)],
                    node("tune_console"), INK_SOFT)

        # The twin path. q1..q3 are part of <ns>/*, so this arrow stays in the
        # producer column instead of doubling back through the topic boxes.
        jsb_up = node("joint_state_bridge")
        js_on = s["js_hz"] > 0
        mode = ("q* direct" if fw["direct"] else "meas* fallback") if jsb_up else ""
        self._arrow(p, [(99, 366), (99, 456)], flowing and jsb_up)
        self._box(p, QRectF(24, 456, 150, 42), "joint_state_bridge", mode,
                  jsb_up and js_on, INK)
        self._box(p, QRectF(24, 514, 150, 38), "mock_arm",
                  "replaying config.h" if mock_up else "",
                  mock_up and js_on, INK_SOFT)
        self._box(p, QRectF(206, 469, 138, 26), "/joint_states", "", js_on, INK)
        self._arrow(p, [(174, 477), (190, 477), (190, 482), (206, 482)],
                    jsb_up and js_on)
        self._arrow(p, [(174, 533), (190, 533), (190, 482), (206, 482)],
                    mock_up and js_on, INK_SOFT)
        self._text(p, QRectF(206, 497, 138, 12), f"{s['js_hz']:.0f} Hz",
                   INK_SOFT if js_on else DIM_TEXT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)

        rsp = node("robot_state_publisher")
        rviz = node("rviz")
        tf_on = s["tf_hz"] > 0
        self._box(p, QRectF(376, 456, 170, 40), "robot_state_publisher", "",
                  rsp and tf_on, INK)
        self._arrow(p, [(344, 482), (376, 482)], js_on and rsp)
        self._arrow(p, [(461, 496), (461, 516)], rsp and tf_on)
        self._text(p, QRectF(466, 498, 90, 14),
                   f"/tf  {s['tf_hz']:.0f} Hz" if tf_on else "/tf",
                   INK_SOFT if tf_on else DIM_TEXT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)
        self._box(p, QRectF(376, 516, 170, 40), "rviz2",
                  "tracking" if (rviz and tf_on) else
                  "no transforms" if rviz else "", rviz and tf_on, INK)

        # The tool trace comes straight from whichever joint source is running.
        src_y = 552 if mock_up else 498
        self._arrow(p, [(99, src_y), (99, 566), (461, 566), (461, 556)],
                    js_on and rviz, INK_SOFT)
        self._text(p, QRectF(206, 550, 140, 12), "/tool_path",
                   INK_SOFT if js_on else DIM_TEXT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)

    def _legend(self, p: QPainter) -> None:
        y = DH - 22
        p.setPen(QPen(DIM, 0.8))
        p.drawLine(QPointF(24, y - 4), QPointF(DW - 24, y - 4))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(INK))
        p.drawEllipse(QRectF(24, y + 4, 6, 6))
        self._text(p, QRectF(36, y, 60, 14), "live", INK_SOFT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)
        p.setBrush(QBrush(DIM))
        p.drawEllipse(QRectF(78, y + 4, 6, 6))
        self._text(p, QRectF(90, y, 90, 14), "idle", INK_SOFT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)
        p.setBrush(QBrush(ALERT))
        p.drawEllipse(QRectF(130, y + 4, 6, 6))
        self._text(p, QRectF(142, y, 90, 14), "error", INK_SOFT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)
        self._text(p, QRectF(190, y, DW - 214, 14),
                   "dashes move where data moves   ·   d = query dynamics "
                   "terms   ·   q = quit", DIM_TEXT, 6.6, False,
                   Qt.AlignmentFlag.AlignLeft)


def main(args=None) -> None:
    rclpy.init(args=args)
    monitor = FlowMonitor()

    spinner = threading.Thread(target=rclpy.spin, args=(monitor,), daemon=True)
    spinner.start()

    app = QApplication(sys.argv)
    # Becomes the Wayland app_id, which is what the Hyprland window rules match.
    app.setDesktopFileName("mini-ranka-flow")
    app.setApplicationName("mini-ranka-flow")

    widget = FlowWidget(monitor)
    widget.show()

    # Qt's event loop swallows SIGINT unless Python gets a chance to run.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    nudge = QTimer()
    nudge.timeout.connect(lambda: None)
    nudge.start(200)

    try:
        app.exec()
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
