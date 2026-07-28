#pragma once

#include "autons.hpp"     // IWYU pragma: keep
#include "field.hpp"      // IWYU pragma: keep
#include "subsystems.hpp" // IWYU pragma: keep

/**
 * Touchscreen console for the brain.
 *
 * Five views, switched from a landing page:
 *   LANDING  team logo + entry points
 *   SELECT   alliance / route / start dropdowns beside an animated preview
 *   EDIT     route builder -- tap the field to drop points
 *   LIVE     real odometry telemetry, with a breadcrumb trail
 *   CONSOLE  scrolling debug log
 *
 * This is the machinery. The routes themselves live in src/autons.cpp -- add
 * one there and it appears in the dropdown, animates in the preview and runs
 * in a match, with nothing to change here.
 *
 * Routes are authored in the RED frame (red Alliance Station on the -X wall).
 * Selecting BLUE mirrors every route across the Y axis, so one route
 * definition drives both alliances.
 *
 * IMPORTANT: this owns the LVGL screen, so do NOT call pros::lcd::initialize()
 * (LLEMU) anywhere in the project -- it builds a competing screen and the two
 * will fight over the display.
 */
namespace auton {

/// Build the UI. Call once from initialize(), after chassis.calibrate().
void init();

/// Index into AUTONS, or AUTON_COUNT for the hand-built Custom route.
int selected();

/// True when selected() means the Custom route rather than an entry in AUTONS.
bool custom_selected();

const char* selected_name();

/// Which side the mirroring is resolved against. Never NEUTRAL.
field::Alliance alliance();

/// Drive the selected route. Call from autonomous(). Blocks until finished.
void run_selected();

// ---------------------------------------------------------------------------
// Running from the screen
//
// The RUN button on the SELECT and DESIGN views, for trying a route on a
// practice field without a competition switch.
// ---------------------------------------------------------------------------

/// Start the selected route on its own task and return immediately.
///
/// Refuses, and returns false, if a route is already running or if the brain is
/// under competition control -- a button on the screen must not be able to
/// drive the robot during a real match.
///
/// It has to be a task: run_selected() blocks for the length of the route, and
/// this is called from a touch event, which runs on the LVGL task. Calling it
/// directly would freeze the screen for the whole route.
bool request_run();

/// Ask a running route to stop. Takes effect at the next step boundary, so a
/// hand-built route stops promptly; a compiled routine only stops once it
/// returns, because there is nowhere inside someone else's function to check.
///
/// This is a convenience, not a safety device. The field disable and the power
/// switch are the safety devices.
void request_stop();

/// True from the moment a route starts driving until it finishes, whether it
/// was started by the screen or by autonomous().
bool running();

// ---------------------------------------------------------------------------
// Cross-task requests
//
// These are called from the autonomous / opcontrol tasks, so they only raise a
// flag. The LVGL timer picks it up and does the real work on the UI task --
// touching widgets from another task races the LVGL daemon.
// ---------------------------------------------------------------------------

/// Switch the screen to live telemetry.
void show_live();

/// Start recording driven motion, or stop and load it into the custom route.
void toggle_record();

bool recording();

// ---------------------------------------------------------------------------
// Debug console
// ---------------------------------------------------------------------------

/// Append a line to the on-screen console. printf formatting; the line is
/// truncated rather than wrapped. Safe to call from any task and before init(),
/// so it can be used from initialize() before the screen exists.
///
/// Lines also go to stdout, which reaches `pros terminal` over the wire.
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

/// Drop every buffered line.
void log_clear();

} // namespace auton
