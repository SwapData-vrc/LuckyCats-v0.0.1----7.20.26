
#include "autons.hpp"

#include "auton_selector.hpp"
#include "field.hpp" 
#include "subsystems.hpp"

#include <cmath>

namespace auton {
namespace {
constexpr double PI = 3.14159265358979;

void score_backwards(double into_inches, double height) {
  lemlib::Pose p = chassis.getPose();
  double t = p.theta * PI / 180.0;

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

void example() {
  logf("example: start");

  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  intake.move(127);

  chassis.moveToPoint(-32, 0, 2000, {}, false);
  pros::delay(300);
  intake.move(0);

  chassis.turnToHeading(90, 1000, {}, false);
  score_backwards(12, 0.30);

  chassis.moveToPoint(-30, 0, 2000, {}, false);

  logf("example: done");
}


void red_toggle() {
chassis.arcade( 127,  0);
pros::delay(100);
  chassis.turnToHeading(180, 1000, {}, false);
chassis.arcade( 127,  0);
pros::delay(200);


  chassis.arcade( 0,  0);
}
void blue_toggle() {

chassis.arcade( 127,  0);
pros::delay(1000);
chassis.arcade( 0,  0);
chassis.arcade( -127,  0);
pros::delay(1500);

  chassis.arcade( 0,  0);



  chassis.arcade( 127,  0);
pros::delay(1000);
chassis.arcade( 0,  0);
chassis.arcade( -127,  0);
pros::delay(1500);

  chassis.arcade( 0,  0);
}

void do_nothing() {
    chassis.arcade( 0,  0);
  pros::delay(10000);
}



}  // anonymous namespace — everything above is private to this file.

/* AUTONS and AUTON_COUNT must be defined here, directly in `auton`, and NOT
   inside the anonymous namespace above.
   autons.hpp declares them `extern`, i.e. external linkage. Anything defined
   in an anonymous namespace gets *internal* linkage, so those definitions are
   a different entity entirely and never satisfy the declarations — which is
   why auton_selector.cpp failed with "undefined reference to auton::AUTONS".
   Keep the closing brace above ahead of these definitions. */
const Auton AUTONS[] = {
    {"null", do_nothing, -0.0f, 0.0f, 90.0f},
    {"-RedToggle", red_toggle, -52.0f, 0.0f, 90.0f},
    {"-BlueToggle", blue_toggle, -52.0f, 0.0f, 0.0f},
};

const int AUTON_COUNT = static_cast<int>(sizeof(AUTONS) / sizeof(AUTONS[0]));

/* Asserted on the sizeof expression, not on AUTON_COUNT. AUTON_COUNT is
   declared `extern` in the header, and GCC will not always accept an
   extern-declared const as a constant expression here — that is the
   "'auton::AUTON_COUNT' was not initialized with a constant expression" error
   your teammate hit. sizeof on the array immediately above is a constant
   expression on every toolchain, so this builds the same everywhere. */
static_assert(sizeof(AUTONS) / sizeof(AUTONS[0]) <= MAX_AUTONS,
              "too many routines for the dropdown buffer");

}  // namespace auton