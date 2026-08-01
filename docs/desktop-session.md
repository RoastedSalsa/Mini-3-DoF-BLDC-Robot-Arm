# Desktop session (Hyprland)

Implementation record for the one-keypress workspace: how it was built, the
commands that set it up, and the environment facts that had to be measured
rather than assumed. Day-to-day usage lives in
[`Mark1/tools/README.md`](../Mark1/tools/README.md); this file is the "why it
looks like this" companion.

Before this, a working session meant four terminals, a manual
`pixi shell -e lyrical` in each, and rebuilding the PlotJuggler layout by hand
every time — `Mark1/tools/` was an empty directory and the repo had no shell
scripts at all.

```
workspace 6 — HDMI-A-1 (AOC 24")            workspace 7 — eDP-1 (laptop)
┌──────────────────────────────────────┐    ┌──────────────────────────────┐
│ ┌──────────┐          ┌────────────┐ │    │                              │
│ │ inputs   │          │ flow panel │ │    │        PlotJuggler           │
│ │ tune     │          │ Qt6, live  │ │    │  layout + ROS2 streamer      │
│ ├──────────┤          └────────────┘ │    │  preloaded in live mode      │
│ │ feedback │   RViz, bare 3D view    │    │                              │
│ │ cmd+log  │   tiled, full workspace │    │                              │
│ └──────────┘   floats render above   │    │                              │
└──────────────────────────────────────┘    └──────────────────────────────┘
   3 floating kitty/Qt windows                 1 tiled XWayland window
```

Nothing about the arm changed. The firmware was not touched, and the only ROS2
behaviour change is one new launch argument — everything else is additive.

---

## What was added

| File | What it is |
|---|---|
| `Mark1/tools/mini-ranka-session` | The launcher. `start` / `stop` / `restart` / `status`, `--mock` / `--live` / `--port` |
| `Mark1/tools/hypr/mini-ranka.conf` | Workspace pinning, window rules, `SUPER+A` binds, autostart. Sourced from `~/.config/hypr/hyprland.conf` |
| `Mark1/tools/README.md` | Usage reference |
| `mini_ranka_bridge/mini_ranka_bridge/console.py` | New node `feedback_console`, executable `feedback` |
| `mini_ranka_bridge/mini_ranka_bridge/flow.py` | New node `flow_monitor`, executable `flow` (PyQt6) |
| `mini_ranka_bridge/config/plotjuggler_layout.xml` | Four-plot PlotJuggler layout |
| `mini_ranka_description/rviz/session.rviz` | Reframed RViz config for the session |

Modified: `setup.py` (two entry points, `config/*.xml` in `data_files`),
`package.xml` (`rcl_interfaces`, `tf2_msgs`, `python_qt_binding` exec deps),
`twin.launch.py` (new `rviz_config` argument), plus README pointers.

One line was added outside the repo — see [Hyprland wiring](#hyprland-wiring).

---

## The two new nodes

### `feedback` — what the MCU said, and what we said to it

`tune` publishes to `<ns>/cmd`; this subscribes to the *same* topic plus
`<ns>/log`, so a tuning session reads back as one timeline instead of a list of
replies with no visible cause:

```
08:26:15.714  > DG
08:26:17.160  < dynamics terms: gravity=on inertia=off centrifugal=off coriolis=off
08:26:21.893  > Q9
08:26:23.314  < usage: D | DG | DI | DC | DK | DA | DN
08:26:24.628  ! malformed: home_angle3.14151.2340  + telemetry line lost (33B)
```

`/rosout` is folded in as a third stream so bridge-level events land in the same
timeline — without it a disconnected arm and an idle one both look like silence.
That subscription **must** use `rclpy.qos.qos_profile_rosout_default`; `/rosout`
is transient-local depth 1000 and a default profile silently fails to match, so
the callback simply never fires.

The `!` line is the `doHome` bug made visible: `main.cpp:211-213` prints
`home_angle<q><sensor>` with no newline and no separator, so the telemetry line
that follows is glued onto it, fails to parse as JSON, and arrives on `<ns>/log`
instead of becoming topics. The console names it rather than rendering it as a
normal reply.

### `flow` — where the data actually stops

A Qt6 panel drawing the whole path, firmware → serial → topics → RViz. Every box
is lit by a **measured rate or value**, never by "the process exists" — a bridge
whose serial port dropped is still a node in the graph, which is exactly the
failure `rqt_graph` cannot show you.

| Element | Lit by |
|---|---|
| Trajectory | `x`/`y`/`z` actually changing — static means a manual `T` target |
| Dyn FF + G/I/C/K | `<ns>/log` parse, cross-checked against whether `ff1..3` are really non-zero |
| J1..J3 | rate on `meas<n>`, colour ramped by \|`err<n>`\| |
| Cartesian FK | `mx`/`my`/`mz` non-zero (the `C` toggle parks them at 0 when off) |
| telemetry_bridge | Hz on `<ns>/t`, port and signal count from `/rosout` |
| joint_state_bridge | badge shows `q* direct` vs `meas* fallback` |
| `/joint_states`, `/tf` | measured publish rate |

Topics are discovered, never assumed: `telemetry_bridge` creates its publishers
lazily per JSON key, and the `pid_tuner` firmware emits a completely different
set (`target`/`meas`/`err`/`vel`) than `main`. The panel enumerates whatever
`Float64` topics appear under the namespace and names the profile it recognises.

It is **passive** — it never writes to `<ns>/cmd`. Firmware feature state has no
telemetry channel (`homed`, `dyn_terms`, `cart_meas_enabled` exist only in
Commander text replies), so those start unknown and fill in as replies scroll
past. Pressing `d` sends a bare `D`, the one command `doDynTerms`
(`main.cpp:245`) treats as report-only — `DG`/`DI`/`DC`/`DK` are XOR toggles and
must never be sent automatically.

One deliberate honesty feature: `M1/M2/M3_TORQUE_CONSTANT = 0.0f`
(`config.h:171-176`) and `torque_to_voltage()` returns 0 for `kt <= 0`
(`Dynamics.cpp:173`), so the feedforward path is a no-op *regardless* of `F`/`D`
state. The panel shows `on, but Kt=0 in config.h -> 0 V` instead of drawing a lit
arrow that carries nothing.

---

## Setup

### 1. Build

```bash
cd Mark1/ros2
pixi run -e lyrical build
```

`-e lyrical` is required. Only that environment is materialised, and the
implicit `default` pixi environment has no ROS in it at all — a bare
`pixi run build` fails to even find the task.

### 2. Hyprland wiring

One line appended to `~/.config/hypr/hyprland.conf`:

```conf
# ── Mini_Ranka arm session ───────────────────────────────
source = ~/Documents/Personal_Projects/Mini_Ranka/Mark1/tools/hypr/mini-ranka.conf
```

**It must be sourced last.** Hyprland processes `source` inline at that point in
the file, and `mini-ranka.conf` uses `$mainMod`, which `hyprland.conf` does not
define until its keybindings section.

Everything else — workspace pinning, window rules, the `SUPER+A` binds and the
`exec-once` autostart — lives in the repo file, so the desktop integration is
versioned with the robot. Apply it with:

```bash
hyprctl reload
hyprctl configerrors        # empty output = clean
hyprctl binds -j | python3 -c "import json,sys; print([b['arg'] for b in json.load(sys.stdin) if b['key']=='A'])"
```

### 3. Run

```bash
Mark1/tools/mini-ranka-session              # auto: live if a board is plugged in
Mark1/tools/mini-ranka-session start --mock
Mark1/tools/mini-ranka-session start --live --port /dev/ttyACM1
Mark1/tools/mini-ranka-session stop | restart | status
```

`SUPER+A` starts, `SUPER+SHIFT+A` stops, and the `exec-once` brings it up on
login. `start` is idempotent — running it again just focuses workspace 6.

---

## Environment facts, and the commands that established them

None of this was assumed. Each command below is the one that produced the fact
it sits under, and they are all worth re-running if the machine changes.

**Compositor, terminal, monitors**

```bash
hyprctl version | head -1                # Hyprland 0.55.4 -> new windowrule block syntax
kitty --version                          # 0.47.1, the only terminal installed
hyprctl monitors -j | python3 -c "
import json,sys
for m in json.load(sys.stdin):
    print(m['name'], m['width'], m['height'], m['x'], m['y'], m['reserved'], m['scale'])"
```

`reserved` is `[left, top, right, bottom]` — waybar shows up as `[0, 28, 0, 0]`.
`HDMI-A-1` sits at `0x0` (the big AOC), `eDP-1` at `1920x0` (the laptop panel).

There is no `jq` on this machine, which is why the launcher parses `hyprctl -j`
with `/usr/bin/python3` throughout.

**Window classes — measured, not guessed**

Launch the app into the unused workspace 6 without stealing focus, then read the
class back:

```bash
hyprctl dispatch exec "[workspace 6 silent] bash probe.sh rviz2"
hyprctl dispatch exec "[workspace 6 silent] bash probe.sh ros2 run plotjuggler plotjuggler -n"
sleep 18
hyprctl clients -j | python3 -c "
import json,sys
for c in json.load(sys.stdin):
    if c['workspace']['id']==6:
        print(repr(c['class']), repr(c['title']), 'xwayland=%s' % c['xwayland'], c['size'])"
```

| App | Class | Transport |
|---|---|---|
| RViz | `rviz2` | XWayland (rviz2 links Qt6 xcb) |
| PlotJuggler | `io.plotjuggler.PlotJuggler` | XWayland |
| flow panel | `mini-ranka-flow` | native Wayland `app_id` |
| consoles | `mini-ranka-inputs`, `mini-ranka-feedback` | `kitty --class` |

The flow panel's `app_id` comes from `QApplication.setDesktopFileName()` in
`flow.py`. PyQt6 in the pixi environment picks the Wayland platform plugin
(`.pixi/envs/lyrical/lib/qt6/plugins/platforms/libqwayland.so` is present), even
though the C++ RViz in the same environment does not.

**ROS2 environment**

```bash
ls Mark1/ros2/.pixi/envs/                # only `default` and `lyrical` are built
pixi shell-hook -e lyrical               # ~2 s; its activation sources install/setup.bash
bash -c 'source env.sh && command -v ros2 && ros2 pkg executables plotjuggler'
```

`plotjuggler` is **not on `PATH`** even inside the environment — the binary lives
at `.pixi/envs/lyrical/lib/plotjuggler/plotjuggler` and only resolves through
`ros2 run plotjuggler plotjuggler`. `/usr/bin/plotjuggler` is a separate system
copy with no ROS2 plugins; never use it.

**Qt bindings and QoS profiles**

```bash
.pixi/envs/lyrical/bin/python -c "
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtGui import QPainter
print('ok')"
.pixi/envs/lyrical/bin/python -c "
import rclpy.qos as q; print([n for n in dir(q) if 'profile' in n.lower()])"
```

PyQt6 6.11.0 is present and `python_qt_binding` resolves to it, which is why
`flow.py` imports through `python_qt_binding` (the ROS-native path, same as rqt)
rather than PyQt6 directly. `rich` and `textual` are **not** installed.

**Hyprland rule field names**

The block-rule vocabulary is not guessable and the error message is the only
documentation that matters. Probe against a class that matches nothing:

```bash
hyprctl keyword windowrule "match:class ^(nonexistent-probe)\$, <field> <value>"
# -> ok                        the field exists
# -> invalid field type <x>    it does not
```

Results worth keeping, because four of the eight guesses were wrong:

| Works | Does **not** |
|---|---|
| `border_color` (2- or 3-stop gradient + angle) | `bordercolor` |
| `no_blur` | `blur`, `noblur` |
| `no_shadow` | `shadow`, `drop_shadow`, `shadow_enabled` |
| `fullscreen_state`, `border_size`, `rounding`, `opacity` | `fullscreenstate` |

`match:workspace = <n>` works, which is what scopes the accent to workspace 6.

**PlotJuggler CLI and plugin schema**

```bash
plotjuggler --help                                  # -n, --layout, --start_streamer
strings .pixi/envs/lyrical/lib/plotjuggler_ros/libDataStreamROS2.so |
  grep -aE 'selected_topics|use_header_stamp|max_array_size|ROS2 Topic Subscriber'
```

That confirmed the streamer's registered name is exactly `ROS2 Topic Subscriber`
and the plugin's saved XML element names, rather than guessing the schema.

---

## Three things that cost real time

### RViz ignores `Hide Left Dock`

`session.rviz` sets `Hide Left Dock: true` / `Hide Right Dock: true` under
`Window Geometry`. Those are the documented keys, the strings are present in the
library —

```bash
strings .pixi/envs/lyrical/lib/librviz_common.so |
  grep -aiE 'Hide Left Dock|fullscreen|QMainWindow State'
```

— and **rviz2 15.2.3 ignores them anyway**. The config loads (the camera framing
and background take effect), the docks stay.

What actually strips the menu bar, toolbar and docks is `rviz2 --fullscreen`.
Left alone that also covers waybar and the floating consoles, because Qt's
`showFullScreen()` sets the real window state. The fix is to tell RViz it is
fullscreen while the compositor keeps tiling it:

```conf
windowrule {
    match:class = ^(rviz2?)$
    workspace = 6 silent
    float = no
    fullscreen_state = 0 2      # internal: none   client: fullscreen
}
```

The rule field name is `fullscreen_state`, not `fullscreenstate`, in this
Hyprland version. Probe rule names against a class that matches nothing:

```bash
hyprctl keyword windowrule "match:class ^(nonexistent-probe)$, fullscreen_state 0 2"
# -> ok        (a wrong name answers "invalid field type ...")
```

Because of this, the session launches RViz itself rather than letting
`twin.launch.py` do it (`rviz:=false`), which also gives RViz its own PID so
`stop` can kill it instead of chasing a grandchild of `ros2 launch`.

### XML forbids `--` inside a comment

The first PlotJuggler layout had this in its header comment:

```
plotjuggler -n -l <this file> --start_streamer "ROS2 Topic Subscriber"
```

A double hyphen is illegal anywhere inside an XML comment. PlotJuggler put up a
modal reading **"Parse error at line 6: error occurred while parsing comment"**
and then started with no layout at all — the failure looks like the layout was
ignored, not rejected. Validate before shipping a layout:

```bash
python3 -c "
import xml.etree.ElementTree as ET
r = ET.parse('Mark1/ros2/src/mini_ranka_bridge/config/plotjuggler_layout.xml').getroot()
print('plots:', [e.get('name') for e in r.iter('DockArea')], 'curves:', len(list(r.iter('curve'))))"
```

Two related notes. A `<previouslyLoaded_Streamer>` element makes PlotJuggler
prompt **"Start Streaming?"** on every load, so the layout deliberately omits it
and the launcher decides with `--start_streamer` instead. And the layout is
loaded only in live mode: in mock nothing publishes `<ns>/*`, so loading it earns
a "can't find one or more curves" modal every start.

### Inline `exec` rules do not stick to RViz

`hyprctl dispatch exec "[workspace 6 silent] rviz2"` does not work. RViz takes
20+ seconds to map its window, by which point Hyprland has given up associating
the window with the process that spawned it — the window lands on whatever
workspace is focused. PlotJuggler, which maps in a couple of seconds, worked fine
the same way, which is exactly what makes this an easy trap.

Persistent `windowrule` blocks keyed on class are the reliable mechanism, and
window placement is two-phase: the rule handles workspace and float state at map
time, then the launcher polls `hyprctl clients -j` for the class and pins the
exact rect with `movewindowpixel exact` / `resizewindowpixel exact`.

A related detail: PlotJuggler's modal dialogs carry the *same* class as its main
window, so the launcher's window matcher accepts only non-floating candidates
when parking a tiled app — otherwise it moves the dialog and leaves the real
window unplaced.

---

## Look

One palette, four surfaces: `ink #eaf4f8` (live/focused), `ink-soft #9fb6c2`
(secondary), `accent #7fd4e8` (pale cyan — something changed or is active),
`steel #3f4b55` (idle), `alert #ff6b5e` (problems only, `#ff9d8f` for warnings).
Cool tones throughout; the original amber read as a different design language
sitting next to the RViz viewport.

It lands in the window borders (`border_color` rules on workspaces 6 and 7),
waybar (`~/.config/waybar/style.css`, palette declared as `@define-color`, old
theme kept at `style.css.bak`), the feedback console (ANSI truecolor in
`console.py`, cyan for state-change replies), and the flow panel — which is
deliberately the strictest, white/steel/red with no cyan at all, because it is
read at a glance over arbitrary 3D geometry.

Waybar's restyle is global; it has no notion of the active workspace, so unlike
the border accent it cannot be scoped to the session.

**PlotJuggler is the one thing that does not follow.** Its theme lives behind
`StyleSheet::theme` in `~/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini`
(`%3A%3A` is the URL-encoded `::`), but the app rewrites that key at startup
from its own state instead of reading it — set it to `dark` with nothing
running, launch, and it is `light` again. Presetting the file does not work;
changing it inside Preferences does. Left alone.

**The accent** is a `border_color` rule scoped with `match:workspace`, so the
session reads as one instrument panel without touching the global amber
(`col.active_border`) anywhere else. Verify it took by sampling the border
pixels of a focused window rather than trusting the reload — `hyprctl` has no
subcommand that lists resolved rules:

```bash
grim out.png
python3 -c "
from PIL import Image
px = Image.open('out.png').convert('RGB')
print(px.getpixel((12, 38)), px.getpixel((10, 300)))"   # -> (234,242,246) (192,206,214)
```

**Console transparency is kitty's, not the compositor's.** `background_opacity`
fades only the terminal background and leaves glyphs fully opaque; Hyprland's
`opacity` fades the whole surface including the text, which turns 7pt type over a
moving 3D scene to mush. So the launcher passes `-o background_opacity=…` and
the window rule pins compositor `opacity = 1.0`. `no_blur` matters here too —
blur frosts the RViz scene behind the consoles into a grey smear, which is the
opposite of seeing through to the arm.

**The flow panel has no window background at all.** `WA_TranslucentBackground`
plus a `paintEvent` that never fills its rect means only blocks and rules are
drawn. That is only half the job: without `border_size = 0`, `rounding = 0`,
`no_blur` and `no_shadow`, the compositor draws its own rounded, blurred,
shadowed rectangle and the "floating" blocks end up sitting on exactly the
window-shaped background they were meant to escape.

The palette is near-monochrome by design — cool white live, steel idle — because
colour has to survive being read over arbitrary 3D geometry. Red is reserved for
real problems, and the joint error indicator is a continuous `INK -> ALERT`
blend rather than three named threshold colours, which removes a hue without
losing the at-a-glance reading.

## Geometry

Computed every run from `hyprctl monitors -j`, never hardcoded. `hyprland.conf`
asks `HDMI-A-1` for `2560x1440@144` but the panel maxes at `1920x1080`, so
Hyprland silently falls back and any baked-in numbers would be wrong the moment
that line is corrected. The dock also gets unplugged, so a missing monitor falls
back to the largest connected one.

```bash
local col_w=$(( mw * 34 / 100 ))    # console column width
local flow_w=$(( mw * 36 / 100 ))   # flow panel width
local flow_h=$(( mh * 62 / 100 ))   # flow panel height
```

At 1920x1052 usable that gives two 652x508 consoles at `12,40` and `12,560`, and
a 691x652 flow panel at `1217,40`.

The pixi environment is materialised once per boot into
`$XDG_RUNTIME_DIR/mini-ranka/env.sh` and regenerated only when `pixi.lock` or
`install/setup.bash` is newer. Every child sources that one file rather than
paying `pixi run` startup five times.

---

## Verification

```bash
cd Mark1/ros2
pixi run -e lyrical build
pixi run -e lyrical bash -c 'colcon test --packages-select mini_ranka_description && colcon test-result --all'
```

Expect `6 tests, 0 errors, 0 failures`.

```bash
Mark1/tools/mini-ranka-session start --mock
Mark1/tools/mini-ranka-session status
```

Expect three floating consoles plus a tiled `rviz2` on workspace 6, and a tiled
`io.plotjuggler.PlotJuggler` filling workspace 7:

```
ws6   mini-ranka-inputs      652x508  at   12,40   float
ws6   mini-ranka-feedback    652x508  at   12,560  float
ws6   mini-ranka-flow        691x652  at 1217,40   float
ws6   rviz2                 1912x1044 at    4,32   tiled
ws7   io.plotjuggler...     1912x1044 at 1924,32   tiled
```

Exercise the console without hardware — the flow panel and `feedback` react to
these exactly as they would to the firmware:

```bash
ros2 topic pub --once /mini_ranka/cmd std_msgs/msg/String "{data: 'DG'}"
ros2 topic pub --once /mini_ranka/log std_msgs/msg/String \
  "{data: 'dynamics terms: gravity=on inertia=off centrifugal=off coriolis=off'}"
```

Teardown should leave nothing behind:

```bash
Mark1/tools/mini-ranka-session stop
pgrep -af 'rviz2|plotjuggler|mini_ranka_bridge/lib|robot_state_publisher'   # expect nothing
```

Screenshots, if you want to check placement without switching workspaces
manually: `grim -o HDMI-A-1 out.png` (grim captures in monitor-local
coordinates, so subtract the monitor's `x` offset when cropping).

---

## Known limitations

- **mock vs live is decided once, at start.** `twin.launch.py` picks one source
  and cannot switch, so plugging the board in afterwards does nothing. The flow
  panel notices and says so; `SUPER+SHIFT+A` then `SUPER+A` restarts in live.
- **The PlotJuggler layout assumes the `main` firmware.** The `pid_tuner` build
  publishes `target`/`meas`/`err`/`vel` and none of the layout's curves resolve.
  The flow panel detects and names the profile; the layout does not.
- **`--start_streamer` still opens the topic picker.** One click on OK. That is
  PlotJuggler's behaviour, not something the launcher can skip.
- **`mock_arm` warns on every trajectory loop:** `IK solution (-0.236, 1.31,
  -2.001) exceeds the config.h travel limits`. J3 overshoots its `-2.0` limit by
  1 mrad. Pre-existing, unrelated to the session, and tracked with the J3 limit
  issue in [digital-twin.md](digital-twin.md).
- **`session.rviz` on its own does not give a bare viewport** — it needs
  `--fullscreen`. Loading it through `ros2 launch ... rviz_config:=<path>` gives
  the normal RViz window with nicer camera framing.
