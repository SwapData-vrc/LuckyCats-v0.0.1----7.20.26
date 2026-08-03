#pragma once

namespace auton {
using AutonFn = void (*)();

struct Auton {
  const char* name;
  AutonFn run;
  float start_x;
  float start_y;
  float start_heading;
};

extern const Auton AUTONS[];
extern const int AUTON_COUNT;

inline constexpr int MAX_AUTONS = 16;
}
