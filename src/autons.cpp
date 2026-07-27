// ===========================================================================
//  AUTONOMOUS ROUTES
//
//  This is the file to edit. Everything below is either a route or a comment
//  about how to write one; nothing else in the project needs touching to add,
//  remove or reorder a route.
//
//  A route added here:
//    - appears in the ROUTE dropdown on the brain
//    - animates in the field preview, exactly as it will be driven
//    - runs for real when autonomous() starts
//
//  Read include/autons.hpp first. It has the coordinate frame, the list of
//  step types, and the reason routes are data rather than functions.
//
//  ---------------------------------------------------------------------------
//  Quick reference
//  ---------------------------------------------------------------------------
//
//    drive(12)          12 inches forward along the current heading
//    drive(-12)         12 inches in reverse
//    turn(90)           turn to face absolute heading 90 (which is +X)
//    go(-42, 30)        turn to face (-42, 30), then drive straight at it
//    arc(-42, 30)       arc to (-42, 30) in one motion
//    intake_in()        rollers in   (front of the robot)
//    intake_out()       rollers out  (back of the robot)
//    intake_stop()
//    claw_grip()        claw closed
//    claw_release()     claw open
//    lift_to(0.45)      lift to 45% of full travel
//    wait(500)          sit still for 500 ms
//    score(0.45)        raise to 45%, eject off the back, return to travel
//    call(0)            run ACTIONS[0]; see the bottom of this file
//
//  Headings: 0 faces +Y, and they increase clockwise. So 90 faces +X, 180
//  faces -Y, 270 faces -X.
//
//  ---------------------------------------------------------------------------
//  Before you trust a route
//  ---------------------------------------------------------------------------
//
//  - Write it in the RED frame. Blue is mirrored for you.
//  - The robot intakes at the FRONT and scores off the BACK, so reverse into
//    every Goal. See the worked example in include/autons.hpp.
//  - Watch it in the preview. The estimate beside the ROUTE dropdown says
//    whether it fits in the period, and turns amber when it does not. It is a
//    rough model with no PID settling in it, so treat "just under" as "over".
//  - The field coordinates in src/field.cpp are estimates, not published
//    numbers. Measure before relying on them.
// ===========================================================================

#include "autons.hpp"

#include "subsystems.hpp" // IWYU pragma: keep -- ACTIONS need the hardware

namespace auton {
namespace {

// ---------------------------------------------------------------------------
// Example route. Delete it once you have written a real one -- it exists so
// there is a compiling pattern to copy, not because it is worth running.
//
// Starts against the red wall, drives out, comes back. It does not score.
// ---------------------------------------------------------------------------

constexpr Step EXAMPLE[] = {
    lift_to(LIFT_TRAVEL), // get the lift clear of the field first
    intake_in(),
    drive(20), // collect whatever is directly in front
    wait(300),
    intake_stop(),
    drive(-14), // back off, staying on our side
};

} // namespace

// ---------------------------------------------------------------------------
// The routes, in the order they appear in the ROUTE dropdown.
//
//   name, steps, count, start x, start y, start heading, budget in ms
//
// Match autonomous is 15000. Programming skills is 60000.
//
// C++ has no empty array, so this table needs at least one entry. If you want
// the robot to do nothing -- which is a real choice when an elimination partner
// runs a full-field route -- that entry is:
//
//   {"Do nothing", nullptr, 0, -52.0f, 0.0f, 90.0f, 15000},
//
//
// A zero-step route is handled everywhere: it previews as a stationary robot
// and autonomous() returns immediately.
// ---------------------------------------------------------------------------

const Auton AUTONS[] = {
    //   name        steps     count             start x   y      heading  budget
    {"Example", EXAMPLE, route_n(EXAMPLE), -52.0f, 0.0f, 90.0f, 15000},
};

const int AUTON_COUNT = static_cast<int>(sizeof(AUTONS) / sizeof(AUTONS[0]));

static_assert(AUTON_COUNT <= MAX_AUTONS, "too many routes for the dropdown buffer");

// ---------------------------------------------------------------------------
// Actions
//
// For anything the step vocabulary cannot say. A route reaches these with
// call(index), where index is the position in the table below.
//
// These run on the autonomous task, so pros::delay() is fine and blocking is
// expected. The preview cannot see inside them: it holds for the expect_ms
// passed to call() and the robot does not move on screen. Keep driving in
// steps so the preview stays honest, and put mechanism logic here.
//
// Example:
//
//   void unfold() {
//     lift.move_absolute(200, 100);
//     pros::delay(400);
//     claw_pivot.move_absolute(0, 100);
//   }
//
//   const ActionFn ACTIONS[] = {unfold};
//
// and then call(0, 600) in a route.
// ---------------------------------------------------------------------------

const ActionFn ACTIONS[] = {nullptr}; // no actions yet; nullptr entries are skipped

const int ACTION_COUNT = static_cast<int>(sizeof(ACTIONS) / sizeof(ACTIONS[0]));

} // namespace auton
