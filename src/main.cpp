#include "main.h"
#include "auton_selector.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp"     // IWYU pragma: keep
#include "subsystems.hpp"     // IWYU pragma: keep

/**
 * The claw follows the lift, and nothing else is allowed to move it.
 *
 * If the lift is 5 inches or more above where it started, the claw points
 * forward (90 degrees clockwise). Otherwise it points down (0 degrees).
 *
 * This runs in its own task so it keeps working in autonomous as well as
 * driver control. If the check lived in the opcontrol loop instead, the claw
 * would stop following the lift during autonomous.
 */
void claw_task(void* param) {
  bool forward = false;

  // How many motor degrees the lift turns for one inch of height.
  double degrees_per_inch = LIFT_TICKS / LIFT_INCHES;

  // Put the claw down to start with, so it is somewhere known.
  claw_pivot.move_absolute(CLAW_DOWN, 100);

  while (true) {
    double inches = lift.get_position() / degrees_per_inch;

    // Only send a new command when the claw actually needs to swap sides.
    // move_absolute starts its move over every time it is called, so sending
    // the same target 50 times a second would make the claw crawl.
    if (!forward && inches >= CLAW_UP_INCHES) {
      forward = true;
      claw_pivot.move_absolute(CLAW_FORWARD, 100);
    } else if (forward && inches < CLAW_DOWN_INCHES) {
      forward = false;
      claw_pivot.move_absolute(CLAW_DOWN, 100);
    }

    pros::delay(20);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 *
 * Note: LLEMU (pros::lcd) is deliberately not used anywhere in this project.
 * The auton selector owns the LVGL screen, and LLEMU would build a competing
 * one on top of it.
 */
void initialize() {
  chassis.calibrate(); // calibrate sensors

  // Zero the lift and the claw pivot where they are sitting right now.
  // Everything that reads a lift height or commands a claw angle is relative to
  // this, so the robot has to be powered on with the lift down and the claw
  // pointing down. Without this tare, "the starting position is zero" would be
  // a hope rather than a fact.
  lift.tare_position();
  claw_pivot.tare_position();

  // Start the claw task. It keeps running after initialize() finishes -- the
  // pros::Task destructor does not stop the task.
  pros::Task claw(claw_task);

  auton::init(); // touchscreen route selector + field preview
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
/// Drives whichever route is selected on the brain. The routes themselves are
/// in src/autons.cpp -- there is nothing to change here to add one.
void autonomous() { auton::run_selected(); }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  // Hold, not coast: a cascade lift on coast sinks under its own weight the
  // moment the stick is released, and a claw that drifts open drops its load.
  lift.set_brake_mode(pros::MotorBrake::hold);
  claw_pivot.set_brake_mode(pros::MotorBrake::hold);
  intake.set_brake_mode(pros::MotorBrake::coast);
  claw_spin.set_brake_mode(pros::MotorBrake::coast);

  while (true) {
    // arcade control scheme

    // get left y and right x positions
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX);

    // ---- manipulator -------------------------------------------------------
    // TODO: confirm these mappings with whoever is driving. The ports they act
    // on are still placeholders -- see src/subsystems.cpp.

    // R1 / R2: intake in / out. The intake is on the front, scoring is off the
    // back, so "out" is what pushes a load into a Goal.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(127);
      claw_spin.move(127);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
      claw_spin.move(-127);
    }

    else {
      intake.move(0);
      claw_spin.move(0);
    }

    // L1 / L2: lift up / down. Held rather than latched, so letting go stops it
    // where it is rather than running it into a hard stop.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
      lift.move(110);
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
      lift.move(-90);
    else
      lift.brake();

    // There is no claw button. claw_task does it all.

    // A toggles route recording: drive the route by hand, press A again, and
    // the driven path lands in the Custom slot as GOTO steps.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      auton::toggle_record();
      master.rumble(auton::recording() ? "." : "..");
    }

    // delay to save resources
    pros::delay(25);
  }
}