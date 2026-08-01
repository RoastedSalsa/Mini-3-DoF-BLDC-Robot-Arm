# Desktop session

`mini-ranka-session` brings the whole arm workspace up as one Hyprland session,
so a working session is one keypress instead of four terminals and a hand-built
PlotJuggler layout.

This file is the usage reference. For how it was built, the setup commands, and
the environment facts that had to be measured rather than assumed, see
[docs/desktop-session.md](../../docs/desktop-session.md).

```
workspace 6  (HDMI-A-1, the big screen)
┌──────────────────────────────────────────────────────┐
│ ┌────────────────┐                 ┌───────────────┐ │
│ │ inputs         │                 │ flow panel    │ │
│ │ tune console   │                 │ live diagram  │ │
│ ├────────────────┤   RViz, bare    └───────────────┘ │
│ │ feedback       │   3D viewport                     │
│ │ MCU replies    │   filling the workspace           │
│ └────────────────┘                                   │
└──────────────────────────────────────────────────────┘

workspace 7  (eDP-1, the laptop panel)
└─ PlotJuggler
```

```bash
mini-ranka-session              # start; live if a board is plugged in, else mock
mini-ranka-session start --mock # never touch serial
mini-ranka-session start --live --port /dev/ttyACM1
mini-ranka-session stop | restart | status
```

Bound to **SUPER+A** / **SUPER+SHIFT+A**, and started on login by an `exec-once`
in `hypr/mini-ranka.conf`.

## What it starts

| | |
|---|---|
| `ros2 launch mini_ranka_bridge twin.launch.py rviz:=false` | `robot_state_publisher` plus either `mock_arm` or `telemetry_bridge` + `joint_state_bridge` |
| `rviz2 --fullscreen -d session.rviz` | started separately — see "RViz chrome" below |
| `ros2 run mini_ranka_bridge tune` | top-left console: Commander commands out |
| `ros2 run mini_ranka_bridge feedback` | bottom-left console: `<ns>/cmd`, `<ns>/log` and `/rosout` interleaved |
| `ros2 run mini_ranka_bridge flow` | the Qt panel: which part of the path is actually carrying data |
| `ros2 run plotjuggler plotjuggler` | workspace 7; layout and streamer are added in live mode only |

Each console runs inside a shell that stays alive, still in the ROS environment,
if the node exits — a crash leaves you at a usable prompt instead of closing.

## Things worth knowing

**mock vs live is decided once, at start.** `twin.launch.py` picks one source and
cannot switch, so plugging the board in afterwards does nothing. The flow panel
notices and says so; `SUPER+SHIFT+A`, `SUPER+A` restarts in live.

**RViz chrome.** `session.rviz` sets `Hide Left Dock` / `Hide Right Dock` and
rviz2 15.2.3 ignores them. What actually strips the menu bar, toolbar and docks
is `--fullscreen`. On its own that would also cover waybar and the consoles, so
`hypr/mini-ranka.conf` pairs it with `fullscreen_state = 0 2`: RViz is told it is
fullscreen, Hyprland keeps tiling it inside the workspace.

**Geometry is computed every run** from `hyprctl monitors -j`, never hardcoded.
`hyprland.conf` asks HDMI-A-1 for 2560x1440 but the panel only does 1920x1080, so
Hyprland silently falls back and any baked-in numbers would be wrong. If the big
monitor is unplugged the layout falls back to whatever is connected.

To change the proportions, edit these three lines in `mini-ranka-session`:

```bash
local col_w=$(( mw * 34 / 100 ))    # console column width
local flow_w=$(( mw * 36 / 100 ))   # flow panel width
local flow_h=$(( mh * 62 / 100 ))   # flow panel height
```

**PlotJuggler layout.** `mini_ranka_bridge/config/plotjuggler_layout.xml` holds
four plots: tracking error, joint angles, commanded vs measured, and Cartesian.
It is loaded only in live mode — in mock nothing publishes `<ns>/*` and the load
would just raise a "can't find one or more curves" modal. Rearranged it in the
GUI? `File > Save Layout` straight back over that path. Careful editing its
comments: XML forbids `--` inside a comment, and PlotJuggler reports that as
"Parse error at line N" and then silently starts with no layout at all.

**The pixi environment** is materialised once per boot into
`$XDG_RUNTIME_DIR/mini-ranka/env.sh` via `pixi shell-hook -e lyrical`, and
regenerated when `pixi.lock` or `install/setup.bash` is newer. Everything the
session starts sources that one file. If the workspace is not built the launcher
refuses to start and tells you to run `pixi run -e lyrical build` — the `-e
lyrical` is required, the default pixi environment has no ROS in it.

**Logs** land in `Mark1/tools/logs/<component>.log`, truncated per session and
gitignored.

## Look

One palette, used everywhere the session touches:

| | | |
|---|---|---|
| `ink` | `#eaf4f8` | live / focused / commands you sent |
| `ink-soft` | `#9fb6c2` | secondary — the arm answering |
| `accent` | `#7fd4e8` | pale cyan: something is active or just changed |
| `steel` | `#3f4b55` | idle |
| `alert` | `#ff6b5e` | problems only, never decoration (`#ff9d8f` for warnings) |

Cool white and steel, one restrained cyan, and a red family that always means
something is wrong. No warm tones — amber read as a different design language
next to the RViz scene.

Applied in four places, all editable:

- **Window borders** on workspaces 6 and 7 — `border_color` rules in
  `hypr/mini-ranka.conf`, scoped with `match:workspace` so the global amber
  (`col.active_border`) still applies everywhere else on the machine.
- **waybar** — `~/.config/waybar/style.css`, with the palette at the top as
  `@define-color` so it can be retuned in one place. The previous amber theme is
  kept verbatim at `style.css.bak`. This one is global rather than per-workspace:
  waybar has no notion of which workspace you are on.
- **The feedback console** — ANSI truecolor in `console.py`. Cyan marks a state
  change (`dynamics terms: …`, `homed offsets: …`), so the lines you were
  waiting for stand out without a warm accent.
- **The flow panel** — the strictest of the four: white, steel and red only, no
  cyan. It is read at a glance over arbitrary 3D geometry, so it gets the fewest
  hues.

**PlotJuggler stays light.** Its theme lives behind `StyleSheet::theme` in
`~/.config/PlotJuggler/io.plotjuggler.PlotJuggler.ini`, but PlotJuggler rewrites
that key at startup from its own state rather than reading it, so presetting the
file does not stick. Change it from inside the app (Preferences) if you want it
dark; that persists properly. Its window border on workspace 7 matches either
way.

**Console transparency comes from kitty, not the compositor.**
`background_opacity` fades only the terminal background and leaves glyphs fully
opaque; Hyprland's `opacity` would fade the text too, so the window rule pins it
to `1.0`. Adjust without editing anything:

```bash
MINI_RANKA_CONSOLE_OPACITY=0.25 Mark1/tools/mini-ranka-session restart
```

**The flow panel has no window background** — it paints onto a translucent
surface (`WA_TranslucentBackground`) so the blocks float over the RViz scene.
That only works if the compositor also skips its own chrome, which is what
`border_size = 0`, `rounding = 0`, `no_blur` and `no_shadow` in the
`mini-ranka-flow-chrome` rule are for. Drop any of them and a blurred, rounded,
shadowed rectangle reappears around the "floating" blocks.

Its palette is deliberately near-monochrome: cool white is live, steel is idle,
and **red means a problem** — a stalled bridge, a firmware/layout mismatch,
feedforward that is on but computing zero, or a joint whose tracking error has
ramped toward the far end. Nothing decorative is red.

## Files

| | |
|---|---|
| `mini-ranka-session` | the launcher |
| `hypr/mini-ranka.conf` | workspace pinning, window rules, keybinds, autostart — sourced from `~/.config/hypr/hyprland.conf` |
| `logs/` | per-component output from the last session |

Window classes the rules match were read off `hyprctl clients -j`, not guessed:
`rviz2` and `io.plotjuggler.PlotJuggler` come through XWayland, `mini-ranka-flow`
is a native Wayland app_id set by `flow.py`, and the two consoles are
`kitty --class`.
