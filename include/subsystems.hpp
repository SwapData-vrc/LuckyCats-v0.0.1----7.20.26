#pragma once

#ifdef LUCKYCATS_SIM
#include <lemlib/api.hpp> // IWYU pragma: keep
#include <main.h>         // IWYU pragma: keep
#else
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"         // IWYU pragma: keep
#endif

#include <cstdint>

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

extern lemlib::Drivetrain drivetrain;

extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::adi::Encoder vertical_encoder;

extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern lemlib::Chassis chassis;

extern pros::MotorGroup lift;
extern pros::Motor claw_pivot;
extern pros::Motor claw_spin;
extern pros::Motor intake;

extern pros::Optical optical_sensor;
// !! UNMEASURED, and in MOTOR degrees -- not mechanism degrees if geared.
const double LIFT_TICKS = 900;   // bottom to top
const double LIFT_TRAVEL = 0.15; // ride height while driving, 0 to 1

// 0 start, 1 -925, 2 -1350. Does not block.
void spinclaw(int position);

// Call every loop, or the claw strains against its target and whines.
void claw_update();

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
