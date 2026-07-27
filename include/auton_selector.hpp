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
