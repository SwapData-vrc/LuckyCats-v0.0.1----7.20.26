#pragma once

#include "field.hpp"      // IWYU pragma: keep
#include "subsystems.hpp" // IWYU pragma: keep

/**
 * Touchscreen console for the brain.
 *
 * Four views, switched from a landing page:
 *   LANDING  team logo + entry points
 *   SELECT   alliance / route / start dropdowns beside an animated preview
 *   EDIT     route builder -- tap the field to drop points
 *   LIVE     real odometry telemetry, with a breadcrumb trail
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

/// Route slots. CUSTOM is built on the brain and lives in RAM only.
enum class Route { FORWARD_10 = 0, BACKWARD_10 = 1, TURN_90 = 2, CUSTOM = 3 };

inline constexpr int ROUTE_COUNT = 4;

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

} // namespace auton
