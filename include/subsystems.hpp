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
// Lift and claw numbers
//
// One copy of each, here. They used to be in autons.cpp and auton_selector.cpp
// as well, and the copies drifted apart.
//
// None of these are measured yet. They are all in MOTOR degrees, so if the lift
// or the claw is geared, they are not mechanism degrees. TODO: measure.
// ---------------------------------------------------------------------------

const double LIFT_TICKS = 900;   // motor degrees from the bottom to the top
const double LIFT_TRAVEL = 0.15; // where the lift rides while driving, 0 to 1

/// Move the claw pivot to one of its three positions: 0 down, 1 forward,
/// 2 further back. The angles themselves are in src/subsystems.cpp.
///
/// This is how another file gets at a function -- declare it here, in the
/// header. Never #include a .cpp file: that pastes the whole file in and the
/// linker then sees two copies of everything in it.
void spinclaw(int position);

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
