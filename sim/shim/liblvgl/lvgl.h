#pragma once

// On the brain, "liblvgl/lvgl.h" is the LVGL that PROS ships. In the simulator
// it is the upstream LVGL 9.2.0 checkout instead.
//
// Angle brackets are required: this file is itself named liblvgl/lvgl.h, so a
// quoted include would resolve to this file and recurse forever.
#include <lvgl.h>
