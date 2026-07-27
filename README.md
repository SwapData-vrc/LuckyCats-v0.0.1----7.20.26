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
| **`src/autons.cpp`** | **the autonomous routes — this is the file you edit** |
| `include/autons.hpp` | the step vocabulary the routes are written in |
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

## Writing autonomous routes

Routes live in [`src/autons.cpp`](src/autons.cpp) and nowhere else. Add one
there and it appears in the ROUTE dropdown, animates in the field preview, and
runs in a match — there is nothing else to edit.

```cpp
constexpr Step MY_ROUTE[] = {
    lift_to(LIFT_TRAVEL),
    intake_in(),
    go(-42, 30),   // drive to a standoff point beside the Goal
    turn(90),      // put the BACK of the robot at the Goal
    drive(-11),    // reverse into it
    score(0.45),   // raise, eject off the back, return to travel height
    drive(8),
    intake_stop(),
};

const Auton AUTONS[] = {
    //  name       steps     count             start x  y     heading  budget
    {"My route", MY_ROUTE, route_n(MY_ROUTE), -52.0f, 8.0f, 90.0f, 15000},
};
```

A route is a list of steps rather than a function full of `chassis.moveToPoint`
calls because two things have to read it: the code that drives the robot, and
the code that animates the preview. A function can only be executed; a list can
be executed *and* drawn. That is what makes the preview show the route that will
actually run instead of a picture that drifts out of date.

The step vocabulary — `drive`, `turn`, `go`, `arc`, `intake_in`, `claw_grip`,
`lift_to`, `wait`, `score`, `call` — is documented in
[`include/autons.hpp`](include/autons.hpp), with a quick reference repeated at
the top of `autons.cpp`.

**Scoring is front-to-back.** The robot intakes at the front and ejects off the
back, so every Goal is approached nose-out, as above. Getting that backwards
reads identically in the step list and fails silently on the field — watch the
preview, which draws which way the robot is facing.

**Routes are written in the red frame** (red Alliance Station on the -X wall).
Selecting Blue mirrors the whole route across the Y axis, so each route is
written once and drives both sides.

### When steps are not enough

`call(n)` runs `ACTIONS[n]`, a plain function at the bottom of `autons.cpp`, so
anything the vocabulary cannot express is still available:

```cpp
void unfold() {
  lift.move_absolute(200, 100);
  pros::delay(400);
  claw_pivot.move_absolute(0, 100);
}

const ActionFn ACTIONS[] = {unfold};
```

and then `call(0, 600)` in a route. The preview cannot see inside a function, so
it holds still for the 600 ms you declared and the robot does not move on
screen. Keep driving in steps and mechanism logic in actions, and the preview
stays honest.

### Route estimate

The number beside the ROUTE caption is how long the route is predicted to take,
and it turns amber and says `OVER` past that route's `budget_ms`. It comes from
the same made-up constant-rate model the preview uses, so read it as "this is
going to be tight", never as a real time — it has no PID settling, no slew, and
no time for anything to go wrong.

It charges nothing for `intake_*`, `claw_*` and `lift_to`, because
`run_selected` issues those and moves straight on rather than waiting.

## Simulator

The UI can be run on Windows without a brain:

```
python sim/build.py run
```

It recompiles the real `src/auton_selector.cpp`, `src/autons.cpp` and
`src/field.cpp` against stubbed `pros::` and `lemlib::` headers, in a 480x240
window — so a route written in `autons.cpp` can be watched without a brain on
the desk.

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
- **There are no real routes yet.** `src/autons.cpp` ships with one trivial
  `Example` so there is a compiling pattern to copy. Write real ones and delete
  it.
- `src/subsystems.cpp` declares the right drive motors as blue (600 RPM) while
  the comment says 200 RPM. One of the two is wrong.
- `LIFT_TICKS` in `include/autons.hpp` is a guess at full cascade travel. Every
  `lift_to` and `score` height in every route is a fraction of it, so it is the
  single number to measure first.
- Claw-pivot open/closed positions are unmeasured guesses.
- Field coordinates in `src/field.cpp` — goals, toggles, loaders — are estimates,
  not published numbers. Measure before writing routes against them.
- `opcontrol()` only drives. The lift, claw and intake have no driver controls,
  so the robot cannot score in a match yet regardless of what autonomous does.

`sim/sim_hw.cpp` mirrors the globals in `src/subsystems.cpp` — including the
`DEVICE_PORTS` manifest the boot check reads — and has to be updated alongside
it whenever wiring changes.

## Third-party

`include/pros/`, `include/liblvgl/`, `include/lemlib/`, `include/fmt/` and
`firmware/` are vendored dependencies, not team code. PROS and LemLib are
MPL-2.0; LVGL and fmt are MIT.
