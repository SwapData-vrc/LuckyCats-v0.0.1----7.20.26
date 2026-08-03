
#include "main.h"
#include "auton_selector.hpp" // IWYU pragma: keep
#include "lemlib/api.hpp"     // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "subsystems.hpp"     // IWYU pragma: keep

int claw_position = 0;
bool lift_full_extend = true;
bool lift_full_descent = true;
double optical_hue = 0.0;
double optical_distance = 0.0;

// Sticks do not always return to exactly zero. Without this the robot creeps
// across the tile with nobody touching the controller.
constexpr int DRIVE_DEADBAND = 5;

void initialize() {
  chassis.calibrate();

  // spinclaw's positions are measured from here, so the robot must be powered
  // on with the lift down and the claw pointing down.
  lift.tare_position();
  claw_pivot.tare_position();

  auton::init();
}

void disabled() { auton::abort(); }

void competition_initialize() {}

void autonomous() { auton::run_selected(); }

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  // The field kills the autonomous task, but LemLib's chassis task does not
  // know that and will keep driving its last motion into the driver period,
  // fighting the sticks. Nothing below can override it -- it has to be cancelled.
  auton::abort();

  // Do NOT re-zero claw_pivot here. In a match opcontrol runs after autonomous,
  // so that would make "position 0" wherever the claw ended up.
  //
  // Hold, not coast: the lift sinks under its own weight on coast.
  lift.set_brake_mode(pros::MotorBrake::hold);
  claw_pivot.set_brake_mode(pros::MotorBrake::hold);
  intake.set_brake_mode(pros::MotorBrake::coast);
  claw_spin.set_brake_mode(pros::MotorBrake::coast);

  while (true) {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    optical_hue = optical_sensor.get_hue();
    optical_distance = optical_sensor.get_proximity();

    if (leftY > -DRIVE_DEADBAND && leftY < DRIVE_DEADBAND) leftY = 0;
    if (rightX > -DRIVE_DEADBAND && rightX < DRIVE_DEADBAND) rightX = 0;

    chassis.arcade(leftY, rightX);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      claw_position = claw_position + 1;
      if (claw_position > 2) claw_position = 0;
      spinclaw(claw_position);
    }

    claw_update();

     if(lift.get_position() > 2026) {
     lift_full_extend = true;
    } else {
      lift_full_extend = false;
    }

    if(lift.get_position() < -100) {
            lift_full_descent = true;
    } else {
      lift_full_descent = false;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(100);
      claw_spin.move(-50);
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-100);
         claw_spin.move(50);
    }

    else {
      intake.move(0);
      claw_spin.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)  && lift_full_extend == false)
      lift.move(110);
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && lift_full_descent == false)
      lift.move(-90);
    else
      lift.brake();

    // Route recorder. A practice-field tool, so it is dead at an event -- a
    // mis-pressed A must not start writing routes mid-match.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) &&
        !pros::competition::is_connected()) {
      auton::toggle_record();
      master.rumble(auton::recording() ? "." : "..");
    }

   /* if(optical_hue < 30 || optical_hue > 330 && color == field::Alliance::RED ) {
      
        claw_spin.move(-50);

    lift.move_absolute(360, 100);
    pros::delay(700);
    claw_spin.move(0);
    }
    else if(optical_hue < 280 && optical_hue >  160 && color == field::Alliance::BLUE) {
      
       claw_spin.move(-50);
    lift.move_absolute(-20, 100);
    pros::delay(700);
      claw_pivot.move_absolute(-400, 1000);
    claw_spin.move(0); 
    }
    else if(optical_hue < 90 && optical_hue >  30) {

     claw_spin.move(-50);
     claw_pivot.move_absolute(-925, 1000);
    lift.move_absolute(-20, 100);
       claw_pivot.move_absolute(-400, 1000);
    pros::delay(700);
    claw_spin.move(0);

    }
    else {
      pros::delay(1);
    }

    */

    // Required. Without it this loop never yields and the selector's LVGL task
    // and the competition task are starved.
    pros::delay(25);
  }
}
