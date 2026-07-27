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

A short title sequence plays at boot; touch anywhere to skip it. It is driven by
the UI timer, not a sleep, so it never delays autonomous starting.

Four screens after that:

- **Landing** — team badge, version, and the way into the other three
- **Run** — pick alliance, route and start position; the field preview animates
  the route, and X / Y / heading are shown over the field
- **Design** — tap the field to drop waypoints and build a route by hand
- **Live** — odometry readout, battery, and route recording

On the field itself:

- **Tap a Toggle** to start the route from that quadrant. The robot slides to
  the new start pose and the START dropdown follows.
- **Hold a Toggle** to cycle which alliance owns that quadrant. Preview only —
  it does not change what the robot does.

The selection survives a reboot: alliance, route, start, blackout state and the
whole custom route are written to `/usd/auton.txt` on the SD card. It is plain
text and safe to edit on a laptop; delete it to reset. With no SD card inserted
the selector just runs with defaults.

### Blackout Mode

The eye icon on the landing page replaces the screen with an idle-looking logo
page, so a team scouting the pit cannot read the selected route off the brain.
The route still runs normally, and autonomous starting will not reveal it.

To leave, tap the eye icon again — it sits in the same corner on the standby
screen. Holding the badge for about half a second also works.

The state is saved, so a brownout mid-event comes back up still hidden. If the
brain looks idle when you expect the selector, that is why.

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
