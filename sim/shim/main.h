#pragma once

// ---------------------------------------------------------------------------
// PROS API stand-in for the desktop simulator.
//
// This shadows the real include/main.h. It declares only what this project
// actually calls -- it is not a general PROS reimplementation. Adding a call to
// robot code that is not stubbed here shows up as a link error, which is the
// intended behaviour: it forces a decision about what the simulator should do.
//
// Definitions live in sim/sim_hw.cpp.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <initializer_list>
#include <thread>
#include <utility>
#include <vector>

namespace pros {

enum class MotorGearset : std::uint8_t { red, green, blue, invalid };
using MotorGears = MotorGearset;

enum class MotorUnits : std::uint8_t { degrees, rotations, counts, invalid };
enum class MotorBrake : std::uint8_t { coast, brake, hold, invalid };

/// Simulated motor. Commands are recorded so the UI can display them; nothing
/// is physically modelled beyond a position that ramps toward its target.
class Motor {
 public:
  explicit Motor(std::int8_t port, MotorGearset gearset = MotorGearset::green);

  void move(int voltage);
  void move_velocity(int velocity);
  void move_absolute(double position, int velocity);
  void brake();
  // Recorded and otherwise ignored -- there is no coasting mass to model here.
  void set_brake_mode(MotorBrake mode);
  void tare_position();

  double get_position() const;
  double get_target_position() const;
  int get_voltage_command() const;
  std::uint8_t get_port() const;

 private:
  std::int8_t port_;
  MotorGearset gearset_;
  int idx_;
};

class MotorGroup {
 public:
  MotorGroup(std::initializer_list<std::int8_t> ports, MotorGearset gearset = MotorGearset::green);

  void move(int voltage);
  void move_velocity(int velocity);
  void move_absolute(double position, int velocity);
  void brake();
  void set_brake_mode(MotorBrake mode);
  void tare_position();

  double get_position() const;
  double get_target_position() const;
  int get_voltage_command() const;
  std::vector<std::int8_t> get_port_all() const;

 private:
  std::vector<std::int8_t> ports_;
  MotorGearset gearset_;
  int idx_;
};

class Imu {
 public:
  explicit Imu(std::uint8_t port);
  void reset(bool blocking = false);
  bool is_calibrating() const;
  double get_heading() const;
  double get_rotation() const;
  std::uint8_t get_port() const;

 private:
  std::uint8_t port_;
};

class Rotation {
 public:
  explicit Rotation(std::int8_t port);
  double get_position() const;
  void reset_position();
  std::uint8_t get_port() const;

 private:
  std::int8_t port_;
};

namespace adi {
class Encoder {
 public:
  Encoder(char port_top, char port_bottom, bool reversed = false);
  std::int32_t get_value() const;
  std::int32_t reset();

 private:
  char top_, bottom_;
  bool reversed_;
};
} // namespace adi

// ---- controller ----------------------------------------------------------

enum controller_id_e_t { E_CONTROLLER_MASTER = 0, E_CONTROLLER_PARTNER = 1 };

enum controller_analog_e_t {
  E_CONTROLLER_ANALOG_LEFT_X = 0,
  E_CONTROLLER_ANALOG_LEFT_Y,
  E_CONTROLLER_ANALOG_RIGHT_X,
  E_CONTROLLER_ANALOG_RIGHT_Y
};

enum controller_digital_e_t {
  E_CONTROLLER_DIGITAL_L1 = 6,
  E_CONTROLLER_DIGITAL_L2,
  E_CONTROLLER_DIGITAL_R1,
  E_CONTROLLER_DIGITAL_R2,
  E_CONTROLLER_DIGITAL_UP,
  E_CONTROLLER_DIGITAL_DOWN,
  E_CONTROLLER_DIGITAL_LEFT,
  E_CONTROLLER_DIGITAL_RIGHT,
  E_CONTROLLER_DIGITAL_X,
  E_CONTROLLER_DIGITAL_B,
  E_CONTROLLER_DIGITAL_Y,
  E_CONTROLLER_DIGITAL_A
};

/// Reads the PC keyboard. See sim/sim_hw.cpp for the key map.
class Controller {
 public:
  explicit Controller(controller_id_e_t id);

  std::int32_t get_analog(controller_analog_e_t channel);
  bool get_digital(controller_digital_e_t button);
  bool get_digital_new_press(controller_digital_e_t button);
  std::int32_t rumble(const char* pattern);

 private:
  controller_id_e_t id_;
  bool prev_[32] = {};
};

// ---- free functions ------------------------------------------------------

void delay(std::uint32_t milliseconds);
std::uint32_t millis();

namespace battery {
double get_capacity();
double get_voltage();
} // namespace battery

namespace competition {
bool is_autonomous();
bool is_disabled();
/// Always false here: there is no field control attached to a PC. The Run
/// button on the screen uses this to refuse to drive during a real match.
bool is_connected();
} // namespace competition

/// Detached std::thread behind a PROS-shaped name. Enough for "run this in the
/// background so the UI keeps painting", which is all the project asks of it.
/// No priorities, no notifications, no join -- add them when something needs
/// them rather than pretending they work.
class Task {
 public:
  template <class F>
  explicit Task(F&& function, const char* = "") {
    std::thread(std::forward<F>(function)).detach();
  }
};

} // namespace pros
