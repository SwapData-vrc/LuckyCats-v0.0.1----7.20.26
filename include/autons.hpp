#pragma once

#include <cstdint>

/**
 * The autonomous vocabulary.
 *
 * Routes are written in src/autons.cpp as lists of Steps. Nothing else in the
 * project needs editing to add one.
 *
 * The reason a route is a list of data rather than a function full of
 * chassis.moveToPoint() calls is that two things have to read it: the code that
 * drives the robot, and the code that animates the preview on the brain screen.
 * A function can only be executed. A list can be executed *and* drawn, which is
 * what makes the preview show the route that will actually run rather than a
 * drawing of it that quietly drifts out of date.
 *
 * If a route needs something this vocabulary cannot express, that is what
 * Kind::CALL and the ACTIONS table are for -- see below.
 *
 * ---------------------------------------------------------------------------
 * Coordinate frame
 * ---------------------------------------------------------------------------
 *
 * LemLib field coordinates, in inches, origin at the centre of the field:
 *
 *      +Y is away from the red Alliance Station
 *      +X is to the right of it
 *      heading 0 faces +Y and increases CLOCKWISE, so 90 faces +X
 *
 * Write every route in the RED frame -- red Alliance Station on the -X wall.
 * Selecting Blue on the brain mirrors the whole route across the Y axis, so a
 * route is written once and drives both sides.
 *
 * ---------------------------------------------------------------------------
 * Scoring side
 * ---------------------------------------------------------------------------
 *
 * This robot intakes at the FRONT and ejects off the BACK. A Goal is therefore
 * approached nose-out:
 *
 *      go(-42, 30)     drive to a standoff point beside the Goal
 *      turn(90)        put the BACK of the robot at the Goal
 *      drive(-11)      reverse into it
 *      score(0.45)     raise, eject off the back, return to travel height
 *      drive(8)        pull away
 *
 * Getting that backwards reads identically in the step list and fails silently
 * on the field. Watch the preview -- it draws which way the robot is facing.
 */
namespace auton {

// ---------------------------------------------------------------------------
// Steps
// ---------------------------------------------------------------------------

enum class Kind : uint8_t {
  DRIVE,  // a = inches along the current heading, negative reverses
  TURN,   // a = absolute heading in degrees
  GOTO,   // a,b = a field point in inches; flag bit 0 = arc instead of turn-then-drive
  INTAKE, // a = +1 in, -1 out, 0 stop
  CLAW,   // a = 1 grip, 0 release
  LIFT,   // a = 0..1 of full travel
  WAIT,   // a = milliseconds
  SCORE,  // a = lift height 0..1; raise, eject off the back, return to travel
  CALL,   // a = index into ACTIONS; b = how long it takes, in ms, for the preview
};

/// GOTO modifier: without it the robot turns to face the point and then drives
/// straight at it; with it the robot arcs to the point in one motion.
constexpr uint8_t F_SWERVE = 1;

struct Step {
  Kind kind;
  float a, b;
  uint8_t flag;
};

// ---------------------------------------------------------------------------
// Builders
//
// Write steps as drive(12) rather than {Kind::DRIVE, 12, 0, 0}. Same struct,
// but a mistyped field cannot silently become a coordinate.
// ---------------------------------------------------------------------------

constexpr Step drive(float inches) { return {Kind::DRIVE, inches, 0.0f, 0}; }
constexpr Step turn(float heading_deg) { return {Kind::TURN, heading_deg, 0.0f, 0}; }

/// Turn to face (x, y), then drive straight at it.
constexpr Step go(float x, float y) { return {Kind::GOTO, x, y, 0}; }

/// Arc to (x, y) in one motion. Faster, less accurate, needs room to swing.
constexpr Step arc(float x, float y) { return {Kind::GOTO, x, y, F_SWERVE}; }

constexpr Step intake_in() { return {Kind::INTAKE, 1.0f, 0.0f, 0}; }
constexpr Step intake_out() { return {Kind::INTAKE, -1.0f, 0.0f, 0}; }
constexpr Step intake_stop() { return {Kind::INTAKE, 0.0f, 0.0f, 0}; }

constexpr Step claw_grip() { return {Kind::CLAW, 1.0f, 0.0f, 0}; }
constexpr Step claw_release() { return {Kind::CLAW, 0.0f, 0.0f, 0}; }

/// height is 0..1 of full cascade travel.
///
/// Named lift_to, not lift: `lift` is the MotorGroup in subsystems.hpp, and a
/// builder by that name would shadow it inside namespace auton -- so an action
/// that said lift.move_absolute(...) would fail to compile with a message about
/// a non-class type, which is not a helpful way to learn this.
constexpr Step lift_to(float height) { return {Kind::LIFT, height, 0.0f, 0}; }

constexpr Step wait(float ms) { return {Kind::WAIT, ms, 0.0f, 0}; }

/// Raise to `height`, run the rollers out the back, come back down to travel.
constexpr Step score(float height) { return {Kind::SCORE, height, 0.0f, 0}; }

/// Run ACTIONS[action]. `expect_ms` is what the preview and the route estimate
/// charge for it -- the preview cannot see inside your function, so tell it.
constexpr Step call(int action, float expect_ms = 500.0f) {
  return {Kind::CALL, static_cast<float>(action), expect_ms, 0};
}

// ---------------------------------------------------------------------------
// Lift
// ---------------------------------------------------------------------------

/// Where the lift sits while driving: clear of the field, not extended.
constexpr float LIFT_TRAVEL = 0.15f;

/// Encoder ticks for a full cascade extension. TODO: measure this on the robot.
/// Every lift() and score() height in every route is a fraction of it, so it is
/// the one number to get right before any route means anything.
constexpr float LIFT_TICKS = 900.0f;

// ---------------------------------------------------------------------------
// The table
// ---------------------------------------------------------------------------

/// Length of a route array, so the AUTONS table reads as a table instead of a
/// column of sizeof division.
template <int N>
constexpr int route_n(const Step (&)[N]) {
  return N;
}

struct Auton {
  const char* name;    // shown in the ROUTE dropdown; keep it under ~18 chars
  const Step* steps;   //
  int count;           //
  float start_x;       // where the robot is placed, RED frame
  float start_y;       //
  float start_heading; //
  uint32_t budget_ms;  // period this route is meant to fit in; the estimate
                       // beside the ROUTE dropdown turns amber past it
};

/// Defined in src/autons.cpp. May be empty -- the selector copes, it just has
/// nothing but the hand-built Custom slot to offer.
extern const Auton AUTONS[];
extern const int AUTON_COUNT;

/// Hard limit, because the dropdown text is built into a fixed buffer.
inline constexpr int MAX_AUTONS = 16;

// ---------------------------------------------------------------------------
// Escape hatch
//
// Anything the step vocabulary cannot express goes in a function here, and the
// route calls it by index. The preview cannot animate it -- it holds for the
// expect_ms you gave call() and the robot does not move on screen -- so keep
// driving in steps and keep mechanism logic in actions.
// ---------------------------------------------------------------------------

using ActionFn = void (*)();

extern const ActionFn ACTIONS[];
extern const int ACTION_COUNT;

} // namespace auton
