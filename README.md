# LuckyCats — V5RC Override (2026-2027)

Competition code for VEX V5 team Lucky Cats. PROS 4 kernel, LemLib chassis,
LVGL 9.2 touchscreen UI.

**Version 0.0.1.** Early. See [Status](#status) before trusting anything here on
a real field.

## Build and upload

Requires the [PROS CLI](https://pros.cs.purdue.edu/v5/getting-started/).

```
pros make          # build
pros upload        # build and send to a connected brain
pros mu            # both, then open the terminal
```

## What's here

| | |
|---|---|
| `src/main.cpp` | competition entry points — `initialize`, `autonomous`, `opcontrol` |
| `src/subsystems.cpp` | motor, sensor and chassis globals; all port numbers live here |
| `src/auton_selector.cpp` | the touchscreen UI |
| `src/field.cpp` | field geometry, coordinates, and route definitions |
| `src/field_img.c` | field render, baked to a C array |
| `src/logo_img.c` | team badge, baked to a C array |
| `include/` | project headers, plus vendored PROS, LemLib and LVGL |
| `firmware/` | prebuilt PROS and LemLib static libraries |
| `sim/` | desktop brain simulator — see [`sim/README.md`](sim/README.md) |

## Touchscreen

Four screens, driven by touch on the brain:

- **Landing** — team badge, version, and the way into the other three
- **Run** — pick alliance, route and start position; the field preview draws the
  robot at the chosen start pose
- **Design** — tap the field to drop waypoints and build a route by hand
- **Live** — odometry readout, battery, and route recording

## Simulator

The UI can be run on Windows without a brain:

```
python sim/build.py run
```

It recompiles the real `src/auton_selector.cpp` and `src/field.cpp` against
stubbed `pros::` and `lemlib::` headers, in a 480x240 window. Useful for layout,
touch targets and route ordering.

It is **not** an emulator and models no physics. Motion is constant-rate — no
PID, no boomerang, no slew. PID gains, odometry drift, IMU behaviour and wheel
geometry are hardware-only. Full detail in [`sim/README.md`](sim/README.md).

## Status

Work in progress. Known placeholders, in rough order of how much they matter:

- **Motor ports for lift, claw and intake are guesses** (`src/subsystems.cpp`).
  Verify against the actual robot before powering on.
- `src/subsystems.cpp` declares the right drive motors as blue (600 RPM) while
  the comment says 200 RPM. One of the two is wrong.
- Lift and claw-pivot target positions are unmeasured guesses.
- Field coordinates in `src/field.cpp` — goals, toggles, loaders — are estimates,
  not published numbers.
- The three built-in routes are throwaway test moves. No real autonomous yet.

`sim/sim_hw.cpp` mirrors the globals in `src/subsystems.cpp` and has to be
updated alongside it whenever wiring changes.

## Third-party

`include/pros/`, `include/liblvgl/`, `include/lemlib/`, `include/fmt/` and
`firmware/` are vendored dependencies, not team code. PROS and LemLib are
MPL-2.0; LVGL and fmt are MIT.
