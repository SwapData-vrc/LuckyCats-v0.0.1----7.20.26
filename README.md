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

Five screens after that:

- **Landing** — team badge, version, port health, and the way into the rest
- **Run** — pick alliance, route and start position; the field preview animates
  the route, and X / Y / heading are shown over the field
- **Design** — tap the field to drop waypoints and build a route by hand
- **Live** — odometry readout, battery, and route recording
- **Console** — scrolling debug log; see [Console](#console)

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

### Console

The fourth landing button opens a scrolling log: boot state, the port check,
every route and alliance change, and — during a run — one line per step with a
timestamp and the total at the end. `Clear` empties it.

Everything also goes to stdout, so `pros terminal` gets the full history rather
than the last 64 lines the on-screen ring is holding.

Anything in the project can write to it:

```cpp
#include "auton_selector.hpp"

auton::logf("lift at %d ticks", ticks);
```

It is safe to call from any task and before `auton::init()`.

## Routes

Eight built-in routes plus the hand-built Custom slot, chosen from the ROUTE
dropdown. All eight are authored in the **red** frame — red Alliance Station on
the -X wall — and selecting Blue mirrors them across the Y axis, so each route
is written once.

| | |
|---|---|
| `AWP - 3 goals` | both red Alliance Goals and the west neutral Short, aiming at `<SC8>` |
| `Alliance goals` | the same two Alliance Goals, without the third-goal leg |
| `North quadrant` | starts on the north wall: neutral Short north, then the north Alliance Goal |
| `South quadrant` | the same, reflected about the X axis |
| `Tall goal` | the centre Tall Goal, full lift extension |
| `Safe - hold side` | take the Pins in front, retreat, never cross the centre |
| `Do nothing` | for when the elimination partner runs a full-field route |
| `Skills (60 s)` | both Alliance Goals via the north Loader, both reachable neutral Shorts, then the Tall Goal |

**Scoring is front-to-back.** The robot intakes at the front and ejects off the
back, so every Goal is approached nose-out: drive to a standoff point, turn to
put the *back* at the Goal, reverse into it, then `Score`. Getting that
backwards is the easiest mistake to make when editing a route, and it looks
correct in the step list either way — watch the preview.

### Route estimate

The number beside the ROUTE caption is how long the route is predicted to take,
and it turns amber and says `OVER` past 15 s (60 s for skills). It comes from
the same made-up constant-rate model the preview uses, so read it as "this is
going to be tight", never as a real time — it has no PID settling, no slew, and
no time for anything to go wrong. `AWP - 3 goals` estimates ~14.9 s.

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
  Verify against the actual robot before powering on. The landing page runs a
  port check at boot and says how many are missing, which is the fastest way to
  find out — but it can only tell you a port is empty, not that it is wired to
  the right mechanism.
- **No route here has been driven on a real field.** They are geometry built on
  the estimated coordinates below, not tuned autonomous. Treat each one as a
  shape to correct.
- `src/subsystems.cpp` declares the right drive motors as blue (600 RPM) while
  the comment says 200 RPM. One of the two is wrong.
- `LIFT_TICKS` in `src/auton_selector.cpp` is a guess at full cascade travel.
  Every lift and score height in every route is a fraction of it, so it is the
  single number to measure first.
- Claw-pivot open/closed positions are unmeasured guesses.
- Field coordinates in `src/field.cpp` — goals, toggles, loaders — are estimates,
  not published numbers.
- Which quadrants sit on each side of the **Autonomous Line** is not confirmed
  against the current game manual, so the `<SC8>` exclusion is not modelled.
  `AWP - 3 goals` stays in the red half throughout, which should be safe either
  way, but check it before relying on the win point.

`sim/sim_hw.cpp` mirrors the globals in `src/subsystems.cpp` — including the
`DEVICE_PORTS` manifest the boot check reads — and has to be updated alongside
it whenever wiring changes.

## Third-party

`include/pros/`, `include/liblvgl/`, `include/lemlib/`, `include/fmt/` and
`firmware/` are vendored dependencies, not team code. PROS and LemLib are
MPL-2.0; LVGL and fmt are MIT.
