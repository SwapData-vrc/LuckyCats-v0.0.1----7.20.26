#pragma once

// ---------------------------------------------------------------------------
// LemLib stand-in for the desktop simulator.
//
// The motion profile is NOT LemLib's. There is no PID, no boomerang, no slew.
// Motions run at a constant rate toward their target, so the simulator answers
// "does this route go where I meant it to" and never "is this tuned". Treat any
// timing you see here as fiction.
//
// Definitions live in sim/sim_hw.cpp.
// ---------------------------------------------------------------------------

#include "main.h" // IWYU pragma: keep

#include <mutex>

namespace lemlib {

struct Pose {
  double x = 0, y = 0, theta = 0;
  Pose() = default;
  Pose(double x_, double y_, double theta_ = 0) : x(x_), y(y_), theta(theta_) {}
};

/// Wheel diameters, in inches.
namespace Omniwheel {
constexpr float NEW_275 = 2.75f;
constexpr float OLD_275 = 2.75f;
constexpr float NEW_275_HALF = 1.375f;
constexpr float NEW_325 = 3.25f;
constexpr float OLD_325 = 3.25f;
constexpr float NEW_325_HALF = 1.625f;
constexpr float NEW_4 = 4.0f;
constexpr float OLD_4 = 4.18f;
constexpr float NEW_4_HALF = 2.0f;
} // namespace Omniwheel

class TrackingWheel {
 public:
  TrackingWheel(pros::Rotation* encoder, float diameter, float offset, float gear_ratio = 1.0f);
  TrackingWheel(pros::adi::Encoder* encoder, float diameter, float offset, float gear_ratio = 1.0f);
  float offset = 0;
};

struct Drivetrain {
  Drivetrain(pros::MotorGroup* left, pros::MotorGroup* right, float track_width, float wheel_diameter,
             float rpm, float horizontal_drift);
  float track_width, wheel_diameter, rpm, horizontal_drift;
};

struct OdomSensors {
  OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
              TrackingWheel* horizontal2, pros::Imu* imu);
};

struct ControllerSettings {
  ControllerSettings(float kP, float kI, float kD, float anti_windup, float small_error,
                     float small_error_timeout, float large_error, float large_error_timeout, float slew);
};

struct ExpoDriveCurve {
  ExpoDriveCurve(float deadband, float min_output, float curve_gain);
};

struct MoveToPointParams {
  bool forwards = true;
  float maxSpeed = 127;
  float minSpeed = 0;
  float earlyExitRange = 0;
};

struct MoveToPoseParams {
  bool forwards = true;
  float horizontalDrift = 0;
  float lead = 0.6f;
  float maxSpeed = 127;
  float minSpeed = 0;
  float earlyExitRange = 0;
};

struct TurnToHeadingParams {
  int direction = 0;
  float maxSpeed = 127;
  float minSpeed = 0;
  float earlyExitRange = 0;
};

/// Heading convention matches LemLib: 0 faces +Y, increasing clockwise, so the
/// forward unit vector is (sin theta, cos theta).
class Chassis {
 public:
  Chassis(Drivetrain& drivetrain, ControllerSettings& lateral, ControllerSettings& angular,
          OdomSensors& sensors, ExpoDriveCurve* throttle_curve = nullptr,
          ExpoDriveCurve* steer_curve = nullptr);

  void calibrate(bool calibrate_imu = true);

  void setPose(float x, float y, float theta, bool radians = false);
  Pose getPose(bool radians = false, bool standard_pos = false);

  void moveToPoint(float x, float y, int timeout, MoveToPointParams params = {}, bool async = true);
  void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params = {},
                  bool async = true);
  void turnToHeading(float theta, int timeout, TurnToHeadingParams params = {}, bool async = true);

  void waitUntilDone();
  void cancelMotion();

  void tank(int left, int right, float deadband = 0);
  void arcade(int throttle, int turn, bool disable_drive_curve = false, float desaturate_bias = 0.5f);

  // ---- simulator-only ----
  /// Straight-line speed, inches/sec.
  static float sim_drive_speed;
  /// Turn rate, degrees/sec.
  static float sim_turn_speed;

 private:
  void step_to(float tx, float ty, bool forwards, int timeout);
  void step_turn(float target_deg, int timeout);

  mutable std::mutex m_;
  double x_ = 0, y_ = 0, theta_ = 0;
};

} // namespace lemlib
