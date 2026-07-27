#pragma once

/**
 * The list of autonomous routines.
 *
 * A routine is an ordinary function full of ordinary LemLib calls. Write it in
 * src/autons.cpp, add a line to the AUTONS table, and it appears in the ROUTE
 * dropdown on the brain and runs when autonomous() starts.
 *
 * ---------------------------------------------------------------------------
 * Coordinate frame
 * ---------------------------------------------------------------------------
 *
 * LemLib field coordinates, in inches, origin at the centre of the field:
 *
 *      +Y is away from the red Alliance Station
 *      +X is to the right of it
 *      heading 0 faces +Y and increases CLOCKWISE, so 90 faces +X
 *
 * ---------------------------------------------------------------------------
 * Start pose
 * ---------------------------------------------------------------------------
 *
 * The selector calls chassis.setPose() with the start pose in the table before
 * calling the routine, so a routine does not have to set it. Call setPose
 * inside the routine if you would rather ignore the table -- last call wins.
 *
 * The START dropdown on the brain overrides the table for testing, which is
 * only useful if the routine is not setting the pose itself.
 *
 * ---------------------------------------------------------------------------
 * Alliance
 * ---------------------------------------------------------------------------
 *
 * There is no automatic mirroring. A routine that has to behave differently by
 * side asks:
 *
 *      if (auton::alliance() == field::Alliance::BLUE) { ... }
 *
 * Mirroring used to happen behind your back, which meant a routine could not be
 * read literally. Now what is written is what runs.
 */
namespace auton {

using AutonFn = void (*)();

struct Auton {
  const char* name;    // shown in the ROUTE dropdown; keep it under ~18 chars
  AutonFn run;         // the routine itself
  float start_x;       // where the robot is placed before run() is called
  float start_y;       //
  float start_heading; //
};

/// Defined in src/autons.cpp.
extern const Auton AUTONS[];
extern const int AUTON_COUNT;

/// Hard limit, because the dropdown text is built into a fixed buffer.
inline constexpr int MAX_AUTONS = 16;

} // namespace auton
