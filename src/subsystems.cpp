#include "subsystems.hpp" // IWYU pragma: keep


// !! UNVERIFIED: both groups are declared blue (600 RPM). The right-hand
// comment used to say 200 RPM, which is a green cartridge -- one of the two was
// wrong and nobody could tell which from reading this file. Open the gearboxes
// and confirm before trusting odometry, because the cartridge, the wheel size
// and the drivetrain RPM below all scale distance directly.
pros::MotorGroup left_motors({-1, 2, -3}, pros::MotorGearset::blue);  // 600 RPM cartridges
pros::MotorGroup right_motors({4, -5, 6}, pros::MotorGearset::blue);  // 600 RPM cartridges

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
// TODO: placeholder ports -- replace with the real wiring. Free ports at time
// of writing: 7, 8, 9, 11-19. Negative port = motor reversed.
// ---------------------------------------------------------------------------

pros::MotorGroup lift({7, -8}, pros::MotorGearset::green); // two lift motors, geared together
pros::Motor claw_pivot(9, pros::MotorGearset::green);        // open / close, high torque
pros::Motor claw_spin(11, pros::MotorGearset::green);      // claw roller: + grip, - release
pros::Motor intake(12, pros::MotorGearset::blue);          // 600 rpm intake: + in, - out

// ---------------------------------------------------------------------------
// Port manifest, mirroring the constructors above. Read by the boot-time health
// check in auton::init(). Ports are listed with the same sign as the
// constructor argument so this reads as a copy of the wiring, not a paraphrase.
//
// The ADI encoder on 'C'/'D' is not a smart port and cannot be probed the same
// way, so it is deliberately absent.
// ---------------------------------------------------------------------------

const DevicePort DEVICE_PORTS[] = {
    {"drive L1", -1, DevKind::MOTOR},  {"drive L2", 2, DevKind::MOTOR},
    {"drive L3", -3, DevKind::MOTOR},  {"drive R1", 4, DevKind::MOTOR},
    {"drive R2", -5, DevKind::MOTOR},  {"drive R3", 6, DevKind::MOTOR},
    {"lift A", 7, DevKind::MOTOR},     {"lift B", -8, DevKind::MOTOR},
    {"claw pivot", 9, DevKind::MOTOR}, {"claw spin", 11, DevKind::MOTOR},
    {"intake", 12, DevKind::MOTOR},    {"imu", 10, DevKind::IMU},
    {"horiz enc", 20, DevKind::ROTATION},
};

const int DEVICE_PORT_COUNT = static_cast<int>(sizeof(DEVICE_PORTS) / sizeof(DEVICE_PORTS[0]));

//TODO: add more subsystems