// ---------------------------------------------------------------------------
// Simulated hardware: the desktop stand-in for src/subsystems.cpp.
//
// The globals below mirror subsystems.cpp exactly -- same ports, same gearsets,
// same geometry -- so that robot code sees an identical hardware description.
// subsystems.cpp itself is NOT compiled into the simulator; this file replaces
// it, because constructing real PROS devices needs a brain.
//
// Motion is a constant-rate kinematic model. There is no PID here and no
// attempt at one: a fake plant would produce confident, meaningless tuning
// numbers. What this does model is geometry and sequencing -- where a route
// ends up, and in what order.
// ---------------------------------------------------------------------------

#include "subsystems.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace {

constexpr float PI_F = 3.14159265358979f;

std::chrono::steady_clock::time_point g_boot = std::chrono::steady_clock::now();

/// Per-motor state, indexed by the handle each Motor/MotorGroup is given.
struct MotorState {
  int voltage = 0;
  double position = 0;
  double target = 0;
  bool position_mode = false;
  double speed = 200.0; // ticks per second while seeking
};

std::mutex g_motor_m;
std::vector<MotorState> g_motors;

int new_motor(MotorState st = {}) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  g_motors.push_back(st);
  return static_cast<int>(g_motors.size()) - 1;
}

float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

} // namespace

namespace sim {

/// True only while the simulator window has focus. Defined in sim_main.cpp.
bool focused();

/// Advance every motor's position toward its target. Called by the UI loop.
void tick_motors(double dt) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  for (MotorState& m : g_motors) {
    if (!m.position_mode) continue;
    const double delta = m.target - m.position;
    const double step = m.speed * dt;
    m.position = (std::fabs(delta) <= step) ? m.target : m.position + (delta > 0 ? step : -step);
  }
}

} // namespace sim

// ===========================================================================
// pros
// ===========================================================================

namespace pros {

Motor::Motor(std::int8_t port, MotorGearset gearset) : port_(port), gearset_(gearset), idx_(new_motor()) {}

void Motor::move(int voltage) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  g_motors[idx_].voltage = voltage;
  g_motors[idx_].position_mode = false;
}

void Motor::move_velocity(int velocity) { move(velocity); }

void Motor::move_absolute(double position, int velocity) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  g_motors[idx_].target = position;
  g_motors[idx_].position_mode = true;
  g_motors[idx_].speed = std::max(1, std::abs(velocity)) * 6.0;
}

void Motor::brake() { move(0); }
// Accepted and discarded: nothing here coasts, so there is no difference to
// model between hold and coast. It exists so robot code compiles unchanged.
void Motor::set_brake_mode(MotorBrake) {}

double Motor::get_position() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].position;
}

double Motor::get_target_position() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].target;
}

int Motor::get_voltage_command() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].voltage;
}

MotorGroup::MotorGroup(std::initializer_list<std::int8_t> ports, MotorGearset gearset)
    : ports_(ports), gearset_(gearset), idx_(new_motor()) {}

void MotorGroup::move(int voltage) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  g_motors[idx_].voltage = voltage;
  g_motors[idx_].position_mode = false;
}

void MotorGroup::move_velocity(int velocity) { move(velocity); }

void MotorGroup::move_absolute(double position, int velocity) {
  std::lock_guard<std::mutex> lk(g_motor_m);
  g_motors[idx_].target = position;
  g_motors[idx_].position_mode = true;
  g_motors[idx_].speed = std::max(1, std::abs(velocity)) * 6.0;
}

void MotorGroup::brake() { move(0); }
void MotorGroup::set_brake_mode(MotorBrake) {}

double MotorGroup::get_position() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].position;
}

double MotorGroup::get_target_position() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].target;
}

int MotorGroup::get_voltage_command() const {
  std::lock_guard<std::mutex> lk(g_motor_m);
  return g_motors[idx_].voltage;
}

Imu::Imu(std::uint8_t port) : port_(port) {}
void Imu::reset(bool) {}
bool Imu::is_calibrating() const { return false; }
double Imu::get_heading() const { return chassis.getPose().theta; }
double Imu::get_rotation() const { return chassis.getPose().theta; }

Rotation::Rotation(std::int8_t port) : port_(port) {}
double Rotation::get_position() const { return 0; }
void Rotation::reset_position() {}

namespace adi {
Encoder::Encoder(char top, char bottom, bool reversed) : top_(top), bottom_(bottom), reversed_(reversed) {}
std::int32_t Encoder::get_value() const { return 0; }
std::int32_t Encoder::reset() { return 0; }
} // namespace adi

// ---- controller: PC keyboard ----------------------------------------------
//
//   W / S          left stick Y
//   Up / Down      right stick Y
//   Space          button A   (route recording)
//
// Held keys give full deflection; there is no analog range on a keyboard, so
// anything that depends on partial stick travel will not be exercised here.

namespace {
// Win32 keyboard state is global, so an unfocused simulator would otherwise
// drive the robot from whatever the user is typing in another window.
// Qualified from the global namespace: this sits inside namespace pros, where
// a bare "sim::" would bind to pros::sim.
bool key_down(int vk) { return ::sim::focused() && (GetAsyncKeyState(vk) & 0x8000) != 0; }

int axis(int pos_vk, int neg_vk) {
  const bool p = key_down(pos_vk), n = key_down(neg_vk);
  if (p == n) return 0;
  return p ? 127 : -127;
}

int vk_for(controller_digital_e_t button) {
  switch (button) {
    case E_CONTROLLER_DIGITAL_A: return VK_SPACE;
    case E_CONTROLLER_DIGITAL_B: return 'B';
    case E_CONTROLLER_DIGITAL_X: return 'X';
    case E_CONTROLLER_DIGITAL_Y: return 'Y';
    case E_CONTROLLER_DIGITAL_L1: return 'Q';
    case E_CONTROLLER_DIGITAL_L2: return 'E';
    case E_CONTROLLER_DIGITAL_R1: return 'R';
    case E_CONTROLLER_DIGITAL_R2: return 'F';
    default: return 0;
  }
}
} // namespace

Controller::Controller(controller_id_e_t id) : id_(id) {}

std::int32_t Controller::get_analog(controller_analog_e_t channel) {
  switch (channel) {
    case E_CONTROLLER_ANALOG_LEFT_Y: return axis('W', 'S');
    case E_CONTROLLER_ANALOG_RIGHT_Y: return axis(VK_UP, VK_DOWN);
    case E_CONTROLLER_ANALOG_LEFT_X: return axis('D', 'A');
    case E_CONTROLLER_ANALOG_RIGHT_X: return axis(VK_RIGHT, VK_LEFT);
    default: return 0;
  }
}

bool Controller::get_digital(controller_digital_e_t button) {
  const int vk = vk_for(button);
  return vk != 0 && key_down(vk);
}

bool Controller::get_digital_new_press(controller_digital_e_t button) {
  const int slot = static_cast<int>(button) & 31;
  const bool now = get_digital(button);
  const bool was = prev_[slot];
  prev_[slot] = now;
  return now && !was;
}

std::int32_t Controller::rumble(const char* pattern) {
  std::printf("[controller] rumble \"%s\"\n", pattern ? pattern : "");
  return 1;
}

void delay(std::uint32_t milliseconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::uint32_t millis() {
  const auto d = std::chrono::steady_clock::now() - g_boot;
  return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(d).count());
}

namespace battery {
// Drifts slowly so the readout visibly updates instead of sitting frozen.
double get_capacity() {
  const double t = millis() / 1000.0;
  return 88.0 - std::fmod(t / 60.0, 20.0);
}
double get_voltage() { return 12800; }
} // namespace battery

namespace competition {
bool is_autonomous() { return false; }
bool is_disabled() { return false; }
} // namespace competition

} // namespace pros

// ===========================================================================
// lemlib
// ===========================================================================

namespace lemlib {

TrackingWheel::TrackingWheel(pros::Rotation*, float, float offset_, float) : offset(offset_) {}
TrackingWheel::TrackingWheel(pros::adi::Encoder*, float, float offset_, float) : offset(offset_) {}

Drivetrain::Drivetrain(pros::MotorGroup*, pros::MotorGroup*, float track, float wheel, float rpm_,
                       float drift)
    : track_width(track), wheel_diameter(wheel), rpm(rpm_), horizontal_drift(drift) {}

OdomSensors::OdomSensors(TrackingWheel*, TrackingWheel*, TrackingWheel*, TrackingWheel*, pros::Imu*) {}

ControllerSettings::ControllerSettings(float, float, float, float, float, float, float, float, float) {}

ExpoDriveCurve::ExpoDriveCurve(float, float, float) {}

float Chassis::sim_drive_speed = 42.0f; // in/s
float Chassis::sim_turn_speed = 240.0f; // deg/s

Chassis::Chassis(Drivetrain&, ControllerSettings&, ControllerSettings&, OdomSensors&, ExpoDriveCurve*,
                 ExpoDriveCurve*) {}

void Chassis::calibrate(bool) {
  std::printf("[chassis] calibrate (simulated, instant)\n");
}

void Chassis::setPose(float x, float y, float theta, bool radians) {
  std::lock_guard<std::mutex> lk(m_);
  x_ = x;
  y_ = y;
  theta_ = radians ? theta * 180.0f / PI_F : theta;
}

Pose Chassis::getPose(bool radians, bool) {
  std::lock_guard<std::mutex> lk(m_);
  return Pose(x_, y_, radians ? theta_ * PI_F / 180.0 : theta_);
}

void Chassis::step_turn(float target_deg, int timeout) {
  constexpr double DT = 0.010;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(m_);
      const float err = wrap180(target_deg - static_cast<float>(theta_));
      if (std::fabs(err) < 0.5f) {
        theta_ = target_deg;
        break;
      }
      const float step = sim_turn_speed * static_cast<float>(DT);
      theta_ += (std::fabs(err) <= step) ? err : (err > 0 ? step : -step);
      theta_ = wrap180(static_cast<float>(theta_));
    }
    pros::delay(10);
  }
}

void Chassis::step_to(float tx, float ty, bool forwards, int timeout) {
  constexpr double DT = 0.010;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);

  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(m_);
      const double dx = tx - x_, dy = ty - y_;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < 0.75) break;

      // Heading 0 faces +Y and grows clockwise, so bearing is atan2(dx, dy).
      float want = static_cast<float>(std::atan2(dx, dy) * 180.0 / PI_F);
      if (!forwards) want = wrap180(want + 180.0f);

      const float herr = wrap180(want - static_cast<float>(theta_));
      const float tstep = sim_turn_speed * static_cast<float>(DT);
      theta_ = wrap180(static_cast<float>(theta_) +
                       ((std::fabs(herr) <= tstep) ? herr : (herr > 0 ? tstep : -tstep)));

      // Only translate once roughly pointed the right way, so the path curves
      // in rather than crabbing sideways.
      if (std::fabs(herr) < 25.0f) {
        const double step = std::min<double>(sim_drive_speed * DT, dist);
        const double th = theta_ * PI_F / 180.0;
        const double dir = forwards ? 1.0 : -1.0;
        x_ += std::sin(th) * step * dir;
        y_ += std::cos(th) * step * dir;
      }
    }
    pros::delay(10);
  }
}

void Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool) {
  step_to(x, y, params.forwards, timeout);
}

void Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool) {
  step_to(x, y, params.forwards, timeout);
  step_turn(theta, 1200);
}

void Chassis::turnToHeading(float theta, int timeout, TurnToHeadingParams, bool) {
  step_turn(theta, timeout);
}

void Chassis::waitUntilDone() {}
void Chassis::cancelMotion() {}

void Chassis::tank(int left, int right, float deadband) {
  if (std::abs(left) < deadband) left = 0;
  if (std::abs(right) < deadband) right = 0;

  // Differential drive on a 10 inch track, integrated at the caller's rate.
  constexpr double DT = 0.025; // opcontrol loop period
  const double track = 10.0;
  const double vl = (left / 127.0) * sim_drive_speed;
  const double vr = (right / 127.0) * sim_drive_speed;

  std::lock_guard<std::mutex> lk(m_);
  const double v = (vl + vr) * 0.5;
  const double omega = (vr - vl) / track; // rad/s, clockwise-positive
  theta_ = wrap180(static_cast<float>(theta_ + omega * DT * 180.0 / PI_F));
  const double th = theta_ * PI_F / 180.0;
  x_ += std::sin(th) * v * DT;
  y_ += std::cos(th) * v * DT;
}

void Chassis::arcade(int throttle, int turn, bool, float) {
  tank(throttle + turn, throttle - turn);
}

} // namespace lemlib

// ===========================================================================
// Hardware globals -- mirrors src/subsystems.cpp
// ===========================================================================

pros::MotorGroup left_motors({-1, 2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, -5, 6}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10, lemlib::Omniwheel::NEW_325, 360, 2);

pros::Imu imu(10);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

pros::MotorGroup lift({7, -8}, pros::MotorGearset::green);
pros::Motor claw_pivot(9, pros::MotorGearset::green);
pros::Motor claw_spin(11, pros::MotorGearset::green);
pros::Motor intake(12, pros::MotorGearset::blue);

// Same manifest as src/subsystems.cpp. Nothing here probes it -- there are no
// smart ports on a PC -- but the symbol has to exist or auton_selector.cpp
// fails to link, and a stale copy would be worse than none.
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
