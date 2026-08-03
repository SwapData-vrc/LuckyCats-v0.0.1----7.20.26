#include "subsystems.hpp" // IWYU pragma: keep

// !! UNVERIFIED: cartridge, wheel size and drivetrain RPM all scale odometry
// distance directly. Open the gearboxes and count the teeth before trusting it.
pros::MotorGroup left_motors({-7, -5}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 6}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_motors,
                              &right_motors,
                              10,                        // track width, inches
                              lemlib::Omniwheel::NEW_325, // wheel
                              360,                       // wheel RPM after gearing
                              2                          // horizontal drift
);

// !! PORT 20 IS CLAIMED TWICE. device_check() only PRINTS this -- nothing
// refuses to start, so the robot will happily run a match with the horizontal
// tracking wheel reading PROS_ERR and odometry garbage. Move one of them; port
// 10 is free.
pros::Imu imu(20);

pros::Rotation horizontal_encoder(20);

pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,
                            nullptr,
                            &horizontal_tracking_wheel,
                            nullptr,
                            &imu
);

// kP, kI, kD, anti-windup, small error (in), small timeout (ms),
// large error (in), large timeout (ms), slew
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);

// same order, but the error ranges are in degrees
lemlib::ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors
);

pros::MotorGroup lift({-18, 2}, pros::MotorGearset::green);
pros::Motor claw_pivot(-3, pros::MotorGearset::green);
pros::Motor claw_spin(11, pros::MotorGearset::green);
pros::Motor intake(9, pros::MotorGearset::blue);

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

double claw_target = 0;
uint32_t claw_move_started = 0;

void spinclaw(int position) {
  if (position == 0) claw_target = 0;
  if (position == 1) claw_target = -925;
  if (position == 2) claw_target = -1350;

  claw_move_started = pros::millis();

  claw_pivot.move_absolute(claw_target, 1000);
}

void claw_update() {
  double error = claw_target - claw_pivot.get_position();
  if (error < 0) error = -error;

  bool arrived = error < 5;
  bool stuck = pros::millis() - claw_move_started > 1500;

  if (arrived || stuck) claw_pivot.move(0);
}



 pros::Optical optical_sensor(1);