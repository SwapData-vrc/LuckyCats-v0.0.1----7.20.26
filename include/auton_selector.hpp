#pragma once

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
 * Routes are authored in the RED frame (red Alliance Station on the -X wall).
 * Selecting BLUE mirrors every route across the Y axis, so one route
 * definition drives both alliances.
 *
 * IMPORTANT: this owns the LVGL screen, so do NOT call pros::lcd::initialize()
 * (LLEMU) anywhere in the project -- it builds a competing screen and the two
 * will fight over the display.
 */
namespace auton {

/**
 * Route slots.
 *
 * The order here is the order of the ROUTE dropdown and the order of the
 * PRESETS table in auton_selector.cpp -- all three have to stay in step, and
 * CUSTOM has to stay last because everything before it indexes PRESETS.
 *
 * The saved-state file stores the route as its integer value, so inserting a
 * slot in the middle silently reinterprets a saved selection. Append instead,
 * or accept that the first boot after the change picks the wrong route.
 */
enum class Route {
  AWP = 0,            // the <SC8> Autonomous Win Point attempt
  ALLIANCE_GOALS = 1, // both Alliance Goals, no midfield exposure
  NORTH_QUAD = 2,     // neutral Short goal north, then the north Alliance Goal
  SOUTH_QUAD = 3,     // mirror of NORTH_QUAD about the X axis
  TALL_GOAL = 4,      // centre Tall Goal, full lift extension
  SAFE = 5,           // grab what is in front, retreat -- never crosses centre
  NONE = 6,           // sit still, for when the partner runs a full-field route
  SKILLS = 7,         // 60 s programming skills
  CUSTOM = 8,         // built on the brain, saved to the SD card
};

inline constexpr int ROUTE_COUNT = 9;

/// Build the UI. Call once from initialize(), after chassis.calibrate().
void init();

Route selected();
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
