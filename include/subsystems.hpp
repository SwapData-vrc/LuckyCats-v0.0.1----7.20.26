#pragma once

// Angle brackets, not quotes -- see the note in field.hpp. The simulator
// substitutes stub PROS and LemLib headers by -I order.
#include <main.h>         // IWYU pragma: keep
#include <lemlib/api.hpp> // IWYU pragma: keep
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
// Port manifest
//
// Every smart port the robot uses, in one list, so the boot-time health check
// can name what is missing instead of reporting "a motor". This is the only
// place the ports appear twice; keep it beside the constructors it mirrors so
// the two are edited together.
// ---------------------------------------------------------------------------

enum class DevKind : std::uint8_t { MOTOR, IMU, ROTATION };

struct DevicePort {
  const char* name;
  std::int8_t port; // signed exactly as passed to the constructor
  DevKind kind;
};

extern const DevicePort DEVICE_PORTS[];
extern const int DEVICE_PORT_COUNT;
