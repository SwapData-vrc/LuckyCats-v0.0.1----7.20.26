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
| **`src/autons.cpp`** | **the autonomous routines — this is the file you edit** |
| `include/autons.hpp` | the routine table the selector reads |
| `src/main.cpp` | competition entry points — `initialize`, `autonomous`, `opcontrol` |
| `src/subsystems.cpp` | motor, sensor and chassis globals; all port numbers live here |
| `src/auton_selector.cpp` | the touchscreen UI |
| `src/field.cpp` | field geometry and coordinates |
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
- **Run** — pick alliance, routine and start position, and start it with RUN;
  X / Y / heading are shown over the field
- **Design** — tap the field to drop waypoints and build a route by hand, then
  RUN it without leaving the screen
- **Live** — odometry readout, battery, and route recording
- **Console** — scrolling debug log; see [Console](#console)

On the field itself:

- **Tap a Toggle** to start from that quadrant. The robot slides to the new
  start pose and the START dropdown follows.
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
every routine and alliance change, and — during a run — whatever the routine
logs, plus the total at the end. `Clear` empties it.

Everything also goes to stdout, so `pros terminal` gets the full history rather
than the last 64 lines the on-screen ring is holding.

Anything in the project can write to it:

```cpp
#include "auton_selector.hpp"

auton::logf("lift at %d ticks", ticks);
```

It is safe to call from any task and before `auton::init()`.

## Writing autonomous routines

Routines live in [`src/autons.cpp`](src/autons.cpp) and nowhere else. Write an
ordinary function, add a line to the table, and it appears in the ROUTE dropdown
on the brain and runs in a match.

```cpp
void red_left() {
  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  intake.move(127);

  // false = block until finished. Without it this returns immediately and the
  // next line runs while the robot is still moving.
  chassis.moveToPoint(-42, 30, 2000, {}, false);

  // Square up so the BACK faces the Goal, then reverse into it.
  chassis.turnToHeading(90, 1000, {}, false);
  score_backwards(11, 0.45);
}

const Auton AUTONS[] = {
    //  name          function   start x   y     heading
    {"Red left", red_left, -52.0f, 8.0f, 90.0f},
};
```

Plain LemLib — `moveToPoint`, `moveToPose`, `turnToHeading`, `follow`,
`waitUntil`, all of it — plus every motor in `subsystems.hpp` and `pros::delay`.
There is no wrapper API and nothing to learn beyond LemLib itself.

`auton::logf("...")` writes to the on-screen console and to `pros terminal`.
Sprinkled through a routine it turns the console into a timeline of what
actually happened, which beats guessing after a match.

**The selector sets the start pose** from the table before calling the routine,
so there is no need to `setPose` first. Call it anyway to ignore the table.

**Nothing is mirrored for you.** A routine that has to differ by side asks
`auton::alliance()` and branches. Mirroring used to happen behind your back,
which meant a routine could not be read literally.

**Scoring is front-to-back.** The robot intakes at the front and ejects off the
back, so reverse into a Goal rather than driving at it. `score_backwards()` in
`autons.cpp` is the pattern.

### Running one from the screen

The **RUN** button in the header of the Run and Design views drives the selected
route without a competition switch, for a practice field.

It takes two taps. The first arms it and the label turns amber and says `GO?`;
the second, within three seconds, starts it. A single tap that makes a robot
drive is a bad idea in a pit, and the arming lapses on its own so a stray touch
cannot leave the robot one tap from moving for the rest of the day.

While a route is running the same button turns red and says `STOP`. Stopping
takes effect at the next step boundary and cancels whatever motion is in flight,
so a hand-built route stops promptly. A compiled routine only stops when it
returns — there is nowhere inside your function to check a flag. **This is a
convenience, not a safety device.** The field disable and the power switch are
the safety devices.

The button refuses to do anything when the brain is under competition control.
Under field control the only thing that starts autonomous is the field.

### Seeing what a routine does

The field preview cannot animate a compiled routine — it is a function, and the
only way to know its path is to run it. The SELECT card says
`compiled routine - run to trace` instead of pretending otherwise.

Running it draws the real thing. Autonomous switches the screen to **Live**,
clears the trail, and leaves a breadcrumb trace of the path the robot actually
took, on the brain and in the simulator alike. In the simulator that is one
keystroke:

```
python sim/build.py run     # then press F1
```

The number beside the ROUTE caption is the wall-clock length of the last run,
amber past 15 s. It says `not run yet` until there is a measurement — it is a
timing of the real thing, not a guess.

### The hand-built route

The Custom slot is different: it is built on the brain by tapping the field in
the Design view, or by driving a path and recording it. That one *is* a list of
steps the selector can read, so it animates in the preview and gets an estimated
duration. It is for quick tests without a laptop, not for match routines.

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
- **There are no real routines yet.** `src/autons.cpp` ships with one trivial
  `Example` so there is a compiling pattern to copy. Write real ones and delete
  it.
- `src/subsystems.cpp` declares the right drive motors as blue (600 RPM) while
  the comment says 200 RPM. One of the two is wrong.
- `LIFT_TICKS` in `include/subsystems.hpp` is a guess at full cascade travel.
  Every lift height is a fraction of it, so it is the single number to measure
  first.
- `LIFT_INCHES` is a guess at how far that actually lifts, in inches. The
  automatic claw switches at 5 inches, and "5 inches" means nothing until this
  is measured with a tape.
- Claw-pivot down/forward positions are unmeasured guesses, and they are in
  MOTOR degrees — if the pivot is geared, 90 at the shaft is not 90 at the claw.
- Field coordinates in `src/field.cpp` — goals, toggles, loaders — are estimates,
  not published numbers. Measure before writing routes against them.
- Driver controls exist but the button mapping is a guess (R1/R2 intake, L1/L2
  lift). Confirm it with whoever is driving. The claw has no button — it is
  automatic (`claw_task` in `src/main.cpp`).
- The drivetrain cartridge, wheel size and RPM in `src/subsystems.cpp` are all
  marked UNVERIFIED. They scale odometry directly — check them before tuning.

`sim/sim_hw.cpp` mirrors the globals in `src/subsystems.cpp` — including the
`DEVICE_PORTS` manifest the boot check reads — and has to be updated alongside
it whenever wiring changes.

## Third-party

`include/pros/`, `include/liblvgl/`, `include/lemlib/`, `include/fmt/` and
`firmware/` are vendored dependencies, not team code. PROS and LemLib are
MPL-2.0; LVGL and fmt are MIT.
