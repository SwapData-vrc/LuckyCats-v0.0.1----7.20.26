#include "subsystems.hpp" // IWYU pragma: keep


// !! UNVERIFIED: both groups are declared blue (600 RPM). The right-hand
// comment used to say 200 RPM, which is a green cartridge -- one of the two was
// wrong and nobody could tell which from reading this file. Open the gearboxes
// and confirm before trusting odometry, because the cartridge, the wheel size
// and the drivetrain RPM below all scale distance directly.
pros::MotorGroup left_motors({-7, -5}, pros::MotorGearset::blue);  // 600 RPM cartridges
pros::MotorGroup right_motors({4, 6}, pros::MotorGearset::blue);  // 600 RPM cartridges

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              // NEW_325 is a 3.25" omni. The comment here used
                              // to say 4", which would be lemlib::Omniwheel::NEW_4.
                              // !! UNVERIFIED -- measure the actual wheel.
                              lemlib::Omniwheel::NEW_325,
                              // Wheel RPM after external gearing, not cartridge
                              // RPM. 360 off a 600 RPM cartridge means a 5:3
                              // reduction. !! UNVERIFIED -- count the teeth.
                              360,
                              2 // horizontal drift is 2 (for now)
);


// imu
pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
//
// The comments here were left over from the LemLib template and described a
// setup this robot does not have -- they said the first wheel was null when it
// is not, and that the second was omitted because we use IMEs when in fact we
// use tracking wheels. Corrected to describe what is actually wired.
lemlib::OdomSensors sensors(&vertical_tracking_wheel,   // vertical tracking wheel 1
                            nullptr,                    // vertical tracking wheel 2 -- only one fitted
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr,                    // horizontal tracking wheel 2 -- only one fitted
                            &imu                        // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
//
// Drive curves go here if they are wanted, as two more arguments:
//
//   lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
//   lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);
//   lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
//                           sensors, &throttle_curve, &steer_curve);
//
// A commented-out version of that used to sit inside the while loop in
// opcontrol(), where uncommenting it would have constructed a second Chassis
// forty times a second. It belongs beside the real one, at file scope.
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// ---------------------------------------------------------------------------
// Manipulator
//
// Ports in use: 1, 2 lift -- 4, 5, 6, 7 drivetrain -- 9 claw pivot --
// 10 IMU -- 11 claw roller -- 12 intake -- 20 rotation.
// Free: 3, 8, 13-19. Negative port = motor reversed.
//
// TODO: claw and intake ports are still guesses. The boot check reports a port
// that is empty or holds the wrong device type, and refuses to start if two
// subsystems claim the same port -- but it cannot tell you a motor is wired to
// the wrong mechanism.
// ---------------------------------------------------------------------------

pros::MotorGroup lift({-1, 2}, pros::MotorGearset::green); // two lift motors, geared together
pros::Motor claw_pivot(3, pros::MotorGearset::green);        // open / close, high torque
pros::Motor claw_spin(-11, pros::MotorGearset::green);      // claw roller: + grip, - release
pros::Motor intake(20, pros::MotorGearset::blue);          // 600 rpm intake: + in, - out

// ---------------------------------------------------------------------------
// Automatic claw
//
// The claw follows the lift, with no driver control at all: pointing down while
// the lift is low, rotated 90 degrees clockwise once the lift is at least 5
// inches above where it started.
//
// Heights are relative to power-on, which is only meaningful because
// initialize() tares both motors. Without that tare, "the starting position is
// zero" would be a hope rather than a fact.
// ---------------------------------------------------------------------------

double lift_height() { return lift.get_position() / LIFT_TICKS_PER_INCH; }

void update_claw_for_lift() {
  // Latched rather than recomputed from scratch, so the trigger and release
  // heights actually give hysteresis instead of both being tested against the
  // same instant.
  static bool forward = false;
  static bool commanded = false; // has the pivot been told anything yet?

  const bool was = forward;
  const double h = lift_height();
  if (!forward && h >= CLAW_TRIGGER_IN) forward = true;
  else if (forward && h < CLAW_RELEASE_IN) forward = false;

  // Only on a change. move_absolute restarts the internal motion profile every
  // time it is called, so re-issuing the same target at 50 Hz would keep the
  // pivot permanently at the start of a profile and it would crawl.
  if (forward == was && commanded) return;
  commanded = true;
  claw_pivot.move_absolute(forward ? CLAW_FORWARD_DEG : CLAW_DOWN_DEG, CLAW_PIVOT_SPEED);
}

namespace {
void claw_daemon(void*) {
  while (true) {
    update_claw_for_lift();
    pros::delay(20);
  }
}
} // namespace

void start_claw_daemon() {
  static pros::Task t(claw_daemon, nullptr, "claw");
  (void)t;
}

// ---------------------------------------------------------------------------
// Device manifest, read by the boot check in auton::init().
//
// Names only. The ports come from the objects themselves at runtime, so this
// cannot disagree with the constructors above.
//
// It used to be a second table of literal port numbers and it drifted the first
// time the wiring changed -- it still called the lift's ports "drive L1" and
// "drive L2" after the drivetrain had moved. A list that can be wrong about the
// hardware is worse than no list at all.
//
// The ADI encoder on 'C'/'D' is not a smart port and cannot be probed, so it is
// deliberately absent.
// ---------------------------------------------------------------------------

const MotorGroupRef MOTOR_GROUPS[] = {
    {"drive left", &left_motors},
    {"drive right", &right_motors},
    {"lift", &lift},
};
const int MOTOR_GROUP_COUNT = static_cast<int>(sizeof(MOTOR_GROUPS) / sizeof(MOTOR_GROUPS[0]));

const MotorRef MOTORS[] = {
    {"claw pivot", &claw_pivot},
    {"claw spin", &claw_spin},
    {"intake", &intake},
};
const int MOTOR_COUNT = static_cast<int>(sizeof(MOTORS) / sizeof(MOTORS[0]));

//TODO: add more subsystems