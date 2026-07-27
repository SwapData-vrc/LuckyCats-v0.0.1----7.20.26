# Brain simulator

Runs the real UI code from `src/auton_selector.cpp` and `src/field.cpp` in a
480x240 window on Windows, so the touchscreen can be worked on without a V5
brain on the desk.

```
python sim/build.py run      # build and launch
python sim/build.py          # build only
python sim/build.py clean
```

First build takes about five minutes (LVGL is ~300 files). After that it is
incremental and takes seconds.

## What this is not

Not an emulator. It does not run `bin/hot.package.bin`, and it never will --
that would mean emulating a Cortex-A9 plus VexOS plus undocumented VEX
peripherals.

Instead the same *source* is compiled for Windows, with `pros::` and `lemlib::`
replaced by stubs. So it answers:

- does the layout fit, and does every screen render
- are the touch targets reachable
- does a route go where it was meant to go, in the right order
- does the trail, recorder and telemetry logic behave
- does a Goal get approached back-first, which is what front-to-back scoring
  needs and what the step list cannot show you

It cannot answer anything about the physical robot. Motion here is
constant-rate with no PID, no boomerang and no slew, because a made-up plant
would produce confident and completely meaningless tuning numbers. **PID gains,
odometry drift, IMU behaviour, motor current and wheel geometry are
hardware-only.** A route that looks perfect here can still drive badly.

## Controls

| | |
|---|---|
| Mouse | touchscreen |
| Tap a Toggle | start the route from that quadrant; the robot slides there |
| Hold a Toggle | cycle who owns that quadrant (preview only) |
| Hold the badge | leave Blackout Mode |
| F1 | run `autonomous()` |
| F2 | run `opcontrol()` |
| W / S | left stick, during opcontrol |
| Up / Down | right stick, during opcontrol |
| Space | controller button A -- toggles route recording |
| Esc | quit |

Keys only register while the simulator window has focus. Win32 keyboard state
is global, so without that gate the simulator reacts to typing in other apps --
which is exactly how an early version quietly killed itself when Esc was pressed
somewhere else.

## Layout

| | |
|---|---|
| `build.py` | build script. A Makefile does not work here: make reads the colon in `C:/...` as a rule separator, and the project path has spaces |
| `mk_lv_conf.py` | regenerates `lv_conf.h` from the upstream LVGL template |
| `lv_conf.h` | generated -- do not edit |
| `shim/main.h` | PROS API stubs |
| `shim/lemlib/api.hpp` | LemLib stubs |
| `shim/liblvgl/lvgl.h` | redirects to the upstream LVGL checkout |
| `sim_hw.cpp` | stub implementations, robot model, and the hardware globals that replace `src/subsystems.cpp` |
| `sim_main.cpp` | window, main loop, competition-mode keys |

The selector persists its state. On the brain that is `/usd/auton.txt` on the SD
card; here it is `luckycats_sim_auton.txt` in the build directory, switched by
the `LUCKYCATS_SIM` define that `build.py` passes. Delete the file to reset.
Editing it by hand is the quickest way to jump straight to a route or alliance
without working the dropdowns.

`LUCKYCATS_SIM` also skips the boot-time port check, which has nothing to probe
here. The console says so rather than reporting everything present.

The toolchain, the LVGL checkout and all object files live in
`%LOCALAPPDATA%\LuckyCatsSim`, outside the project on purpose: the project sits
in OneDrive, and building in-tree would sync about a gigabyte to the cloud.

`src/subsystems.cpp` is deliberately **not** compiled -- `sim_hw.cpp` defines
the same globals with the same ports and geometry, because constructing real
PROS devices requires a brain. Keep the two in sync when wiring changes.

## Fidelity notes

`lv_conf.h` matches the brain where it matters, in particular
`LV_CACHE_DEF_SIZE 0`. The brain has no image cache, so `lv_image` widgets
re-decode on every invalidate; raising it here would hide that cost and make a
slow screen look fine.

Deliberate differences: `LV_USE_OS`/`LV_USE_WINDOWS` for the window, and louder
logging, because a warning in the console beats a blank screen.

## Adding an API

If robot code calls something not stubbed, it fails at link time with an
undefined reference. That is intentional -- it forces a decision about what the
simulator should do with that call rather than silently pretending.

## Include style

`include/field.hpp` and `include/subsystems.hpp` use angle brackets for
`liblvgl`, `main.h` and `lemlib`. Quoted includes search the including file's
own directory first, which would always find PROS's copies sitting next to
those headers, leaving no way for the simulator to substitute its own. The
project `Makefile` adds `-I$(INCDIR)` to `EXTRA_CFLAGS`/`EXTRA_CXXFLAGS` so the
brain build still resolves them -- `common.mk` only passes `-iquote`.
