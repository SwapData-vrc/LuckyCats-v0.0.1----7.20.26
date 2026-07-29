#pragma once

// Quoted for the brain, angle brackets for the simulator -- see the note in
// field.hpp for why this is not just a style choice.
#ifdef LUCKYCATS_SIM
#include <lemlib/api.hpp> // IWYU pragma: keep
#include <main.h>         // IWYU pragma: keep
#else
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"         // IWYU pragma: keep
#endif

#include <cstdint>

// Declarations only -- every one of these is defined exactly once in
// src/subsystems.cpp. Include this header (never subsystems.cpp itself)
// from any file that needs the robot's hardware or chassis.

// drivetrain motors
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// drivetrain settings
extern lemlib::Drivetrain drivetrain;

// sensors
extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::adi::Encoder vertical_encoder;

// tracking wheels
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

// odometry settings
extern lemlib::OdomSensors sensors;

// PID controllers
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

// the chassis
extern lemlib::Chassis chassis;

// ---------------------------------------------------------------------------
// Manipulator
//
// !! PORTS ARE PLACEHOLDERS -- set these to the real wiring before flashing. !!
// Ports already taken: 1-6 drivetrain, 10 IMU, 20 rotation, ADI C/D encoder.
// ---------------------------------------------------------------------------

extern pros::MotorGroup lift;      // two lift motors
extern pros::Motor claw_pivot;     // opens / closes the claw
extern pros::Motor claw_spin;      // claw roller: grip (+) / release (-)
extern pros::Motor intake;         // 600 rpm intake: in (+) / out (-)

// ---------------------------------------------------------------------------
// Lift and claw
//
// One copy of these numbers. They used to live in both src/autons.cpp and
// src/auton_selector.cpp, which is the same duplicate-source-of-truth trap that
// put a drivetrain motor in the lift group.
//
// !! All of it is in MOTOR shaft units, and none of it is measured. If the lift
// or the claw pivot is geared, shaft degrees are not mechanism degrees.
// ---------------------------------------------------------------------------

/// Encoder ticks for a full cascade extension. TODO: measure.
constexpr double LIFT_TICKS = 900.0;

/// How far the carriage actually rises over that full extension, in inches.
/// TODO: measure -- run the lift bottom to top and put a tape on it. Everything
/// expressed in inches below is only as right as this number.
constexpr double LIFT_FULL_TRAVEL_IN = 24.0;

/// Derived, so the two cannot drift apart. On a cascade lift the carriage moves
/// several times the ratio of a single stage, which is exactly why this is
/// measured end to end rather than computed from sprocket teeth.
constexpr double LIFT_TICKS_PER_INCH = LIFT_TICKS / LIFT_FULL_TRAVEL_IN;

/// Where the lift sits while driving, as a fraction of full travel: clear of
/// the field, not extended.
constexpr double LIFT_TRAVEL = 0.15;

/// Claw pivot angles, in motor degrees from its position at power-on.
/// TODO: measure. 90 shaft degrees is only 90 claw degrees if it is direct
/// drive.
constexpr double CLAW_DOWN_DEG = 0.0;     // stowed, pointing down
constexpr double CLAW_FORWARD_DEG = 90.0; // deployed, pointing forward, +90 clockwise

/// The rule: at least 5 inches above the starting height and the claw points
/// forward, otherwise it points down.
constexpr double CLAW_TRIGGER_IN = 5.0;

/// It drops back at 4.5 rather than 5 so that a lift parked exactly on the
/// threshold cannot rattle the pivot back and forth every 25 ms. A single
/// shared number would read cleaner and behave worse.
constexpr double CLAW_RELEASE_IN = 4.5;

constexpr int CLAW_PIVOT_SPEED = 100;

/// Lift height in INCHES, measured from where it sat at power-on.
double lift_height();

/// Point the claw forward when the lift is at least CLAW_TRIGGER_IN above its
/// starting height, and down when it is not. Idempotent -- it only touches the
/// motor when the state actually flips.
///
/// Normally there is no reason to call this: start_claw_daemon() runs it. It is
/// still exposed so a routine can force an immediate update.
void update_claw_for_lift();

/// Start the background task that keeps the claw following the lift, in every
/// mode -- autonomous, driver control, and while a route runs from the screen.
/// Called once from initialize(). Calling it again does nothing.
///
/// It is a task rather than a call in each loop because there is no single loop
/// that covers every mode, and a claw that only follows the lift during driver
/// control is worse than one that never does: it works in practice and fails in
/// a match.
void start_claw_daemon();

// ---------------------------------------------------------------------------
// Device manifest
//
// Names paired with the objects themselves, NOT with port numbers. The boot
// check asks each object what port it is on, so this can never disagree with
// the constructors above.
//
// It used to be a second table of literal port numbers, which drifted the first
// time the wiring changed: the drivetrain moved and the manifest still called
// the lift's ports "drive L1" and "drive L2". A list that can be wrong about
// hardware is worse than no list.
// ---------------------------------------------------------------------------

struct MotorGroupRef {
  const char* name;
  pros::MotorGroup* group;
};

struct MotorRef {
  const char* name;
  pros::Motor* motor;
};

extern const MotorGroupRef MOTOR_GROUPS[];
extern const int MOTOR_GROUP_COUNT;

extern const MotorRef MOTORS[];
extern const int MOTOR_COUNT;
