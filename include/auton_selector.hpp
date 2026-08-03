#pragma once

#include "autons.hpp"     // IWYU pragma: keep
#include "field.hpp"      // IWYU pragma: keep
#include "subsystems.hpp" // IWYU pragma: keep

// The alliance chosen on the selector screen: RED, BLUE, or NEUTRAL before a
// choice is made. Updated the moment the dropdown changes, and restored from
// the saved state at boot.
extern field::Alliance color;

namespace auton {
void init();

int selected();

bool custom_selected();

const char* selected_name();

field::Alliance alliance();

void run_selected();

// Stops a LemLib motion and any mechanism left running when the field cut
// autonomous off, and clears the "running" flag. Safe to call at any time.
void abort();

// Refuses while under competition control -- a screen button must never be able
// to drive the robot at an event. Returns false if it refused.
bool request_run();

// A convenience, not a safety device. The field disable and the power switch
// are the safety devices.
void request_stop();

bool running();

void show_live();

void toggle_record();

bool recording();

void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

void log_clear();
}
