// ===========================================================================
//  AUTONOMOUS ROUTINES
//
//  This is the file to edit. Write a function, add it to the AUTONS table at
//  the bottom, and it shows up in the ROUTE dropdown on the brain.
//
//  Plain LemLib. Everything in the LemLib API is available here -- moveToPoint,
//  moveToPose, turnToHeading, follow, swingToHeading, waitUntil, waitUntilDone,
//  the lot -- plus every motor in subsystems.hpp and pros::delay.
//
//  ---------------------------------------------------------------------------
//  Things worth knowing
//  ---------------------------------------------------------------------------
//
//  - Field coordinates, inches, origin at field centre. Heading 0 faces +Y and
//    increases clockwise, so 90 faces +X. See include/autons.hpp.
//
//  - The selector sets the start pose from the table before calling you, so
//    there is no need to call chassis.setPose() first. Call it anyway if you
//    want to ignore the table.
//
//  - Motion calls are ASYNCHRONOUS unless the last argument is false. Either
//    pass false, or call chassis.waitUntilDone() before doing anything that
//    depends on having arrived. This is the single easiest way to write a
//    routine that works in testing and fails on a cold field.
//
//  - This robot intakes at the FRONT and ejects off the BACK, so reverse into
//    a Goal rather than driving at it. See score_backwards() below.
//
//  - Nothing is mirrored for you. If a routine needs to differ by side, ask
//    auton::alliance() and branch.
//
//  - auton::logf("...") writes to the on-screen console and to `pros terminal`.
//    Sprinkle it through a routine and the console becomes a timeline of what
//    actually happened, which beats guessing after a match.
//
//  - The field coordinates in src/field.cpp are estimates, not published
//    numbers. Measure before writing routines against them.
// ===========================================================================

#include "autons.hpp"

#include "auton_selector.hpp" // logf, alliance
#include "subsystems.hpp"     // chassis, lift, intake, claw_pivot, claw_spin

#include <cmath>

namespace auton {
namespace {

/// M_PI is a POSIX extension, not standard C++, and is not always visible.
constexpr double PI = 3.14159265358979;

// ---------------------------------------------------------------------------
// Shared helpers. Ordinary functions -- add whatever you find yourself
// repeating.
//
// The claw does not move by itself. If a routine needs it somewhere, call
// spinclaw(0), spinclaw(1) or spinclaw(2) -- down, forward, further back.
// ---------------------------------------------------------------------------

/// Reverse into a Goal and eject. Assumes the robot is already squared up with
/// its back to the Goal -- turnToHeading first. `height` is 0 to 1.
void score_backwards(double into_inches, double height) {
  lemlib::Pose p = chassis.getPose();
  double t = p.theta * PI / 180.0;

  // Heading 0 is +Y and increases clockwise, so forward is (sin, cos).
  chassis.moveToPoint(p.x - std::sin(t) * into_inches, p.y - std::cos(t) * into_inches, 1500,
                      {.forwards = false}, false);

  lift.move_absolute(height * LIFT_TICKS, 100);
  pros::delay(500);

  intake.move(-127);
  claw_spin.move(-127);
  pros::delay(600);
  intake.move(0);
  claw_spin.move(0);

  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  pros::delay(400);
}

// ---------------------------------------------------------------------------
// Routines
//
// Delete the example once you have written a real one. It exists so there is a
// compiling pattern to copy, not because it is worth running.
// ---------------------------------------------------------------------------

void example() {
  logf("example: start");

  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  intake.move(127);

  // false = block until finished. Without it this returns immediately and the
  // next line runs while the robot is still moving.
  chassis.moveToPoint(-32, 0, 2000, {}, false);
  pros::delay(300);
  intake.move(0);

  // Square up so the BACK faces the neutral Short goal at (-40, 0), then
  // reverse into it. Heading 90 faces +X, so the back faces -X.
  chassis.turnToHeading(90, 1000, {}, false);
  score_backwards(12, 0.30);

  chassis.moveToPoint(-30, 0, 2000, {}, false);

  logf("example: done");
}

} // namespace

// ---------------------------------------------------------------------------
// The table, in the order the routines appear in the ROUTE dropdown.
//
//   name, function, start x, start y, start heading
//
// C++ has no empty array, so this needs at least one entry. A routine with an
// empty body is a perfectly good "do nothing", which is a real choice when an
// elimination partner runs a full-field route.
// ---------------------------------------------------------------------------

const Auton AUTONS[] = {
    {"Example", example, -52.0f, 0.0f, 90.0f},
};

const int AUTON_COUNT = static_cast<int>(sizeof(AUTONS) / sizeof(AUTONS[0]));

static_assert(AUTON_COUNT <= MAX_AUTONS, "too many routines for the dropdown buffer");

} // namespace auton
