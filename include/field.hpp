#pragma once

// Angle brackets, not quotes: the desktop simulator (sim/) substitutes its own
// LVGL via -I order, and a quoted include would always resolve to the copy
// sitting next to this header instead.
#include <liblvgl/lvgl.h> // IWYU pragma: keep
#include <cstdint>

/**
 * V5RC Override (2026-2027) field model.
 *
 * Field coordinates match LemLib -- origin at field centre, +Y away from the
 * driver station, +X right, heading 0 faces +Y and increases clockwise.
 *
 * This models the field's fixed structure only. Cups and Pins are deliberately
 * not simulated -- the preview is a driving aid, not a match simulator.
 *
 * Field facts encoded here (confirmed against VEX sources):
 *   - 9 Goals: 4 neutral Short, 1 neutral Tall, 2 red Alliance, 2 blue Alliance
 *   - 4 Toggles, one at the centre of each field wall
 *   - 4 Loaders, two adjacent to each Alliance Station
 *   - The diagonals split the field into 4 Quadrants, each owning one Toggle
 *
 * !! Exact object positions are NOT published in a machine-readable form. Every
 * coordinate lives in the tables at the top of field.cpp, in inches from field
 * centre. Correct those numbers -- nothing else needs to change. !!
 */
namespace field {

// ---------------------------------------------------------------------------
// Scale
// ---------------------------------------------------------------------------

constexpr int PX = 232;
constexpr float SIZE_IN = 144.0f;
constexpr float HALF_IN = SIZE_IN * 0.5f;
constexpr float PX_PER_IN = PX / SIZE_IN;
constexpr float PI_F = 3.14159265358979f;

inline float px_x(float x_in) { return PX * 0.5f + x_in * PX_PER_IN; }
inline float px_y(float y_in) { return PX * 0.5f - y_in * PX_PER_IN; }
inline float in_x(float p) { return (p - PX * 0.5f) / PX_PER_IN; }
inline float in_y(float p) { return (PX * 0.5f - p) / PX_PER_IN; }

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

namespace draw {
void line(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, int width,
          lv_opa_t opa = LV_OPA_COVER, int dash_w = 0, int dash_gap = 0);
void rect(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, lv_opa_t opa = LV_OPA_COVER,
          int radius = 0);
void rect_outline(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, int width,
                  lv_opa_t opa = LV_OPA_COVER, int radius = 0);
void tri(lv_layer_t* l, float ax, float ay, float bx, float by, float cx, float cy, uint32_t color,
         lv_opa_t opa = LV_OPA_COVER);
void disc(lv_layer_t* l, float cx, float cy, float r, uint32_t color, lv_opa_t opa = LV_OPA_COVER);
/// half-disc: side -1 = left of the split axis, +1 = right
void half_disc(lv_layer_t* l, float cx, float cy, float r, int side, uint32_t color,
               lv_opa_t opa = LV_OPA_COVER);
void rot_rect(lv_layer_t* l, float cx, float cy, float w, float h, float sin_t, float cos_t, uint32_t color,
              lv_opa_t opa = LV_OPA_COVER);
} // namespace draw

// ---------------------------------------------------------------------------
// Alliances and quadrants
// ---------------------------------------------------------------------------

enum class Alliance : uint8_t { NEUTRAL = 0, RED = 1, BLUE = 2 };

/// The diagonals cut the field into four wall-adjacent triangles.
enum class Quad : uint8_t { NORTH = 0, SOUTH = 1, EAST = 2, WEST = 3 };

Quad quad_of(float x, float y);

/// Toggle ownership, indexed by Quad. Flipping one recolours that quadrant's
/// yellow pins, exactly as the real Toggle does.
extern Alliance toggle_owner[4];

void cycle_toggle(Quad q);

// ---------------------------------------------------------------------------
// Goals
// ---------------------------------------------------------------------------

enum class GoalKind : uint8_t { SHORT, TALL, ALLIANCE };

struct Goal {
  float x, y;
  float half_w, half_h;
  GoalKind gkind;
  Alliance alliance;
  float min_lift; // 0..1 lift height needed to score here
};

constexpr int GOAL_COUNT = 9;
extern const Goal goals[GOAL_COUNT];

// ---------------------------------------------------------------------------
// Toggles and loaders
// ---------------------------------------------------------------------------

struct Toggle {
  float x, y;
  Quad quadrant; // NB: not "quad" -- newlib's sys/types.h defines that
  bool horizontal; // drawn as a roller along the wall
};

extern const Toggle toggles[4];

/// Half-extents of a drawn Toggle, in inches. Sized for a fingertip rather than
/// to scale: on a 232 px field a true-size roller is about 4 px, which is not
/// selectable on a resistive touchscreen with a glove on.
constexpr float TOGGLE_LONG_IN = 7.0f;
constexpr float TOGGLE_THICK_IN = 2.6f;

/// Tap radius for "the user meant this Toggle", in inches. Deliberately much
/// larger than the drawn art -- misses are far more annoying than overshoot,
/// and nothing else on the field competes for this area.
constexpr float TOGGLE_HIT_IN = 17.0f;

struct Loader {
  float x, y;
  Alliance alliance;
};

extern const Loader loaders[4];

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

/// Tiles, quadrant diagonals, loaders, goals. Static -- safe to cache.
void draw_field_static(lv_layer_t* l);

/// Toggles, plus a wash over each quadrant showing who owns it. Separate from
/// the static art because flipping a toggle changes both.
///
/// highlight is a Quad index, or -1 for none: that quadrant is drawn as the
/// currently chosen start, with a ring whose radius follows `pulse` (0..1).
void draw_toggles(lv_layer_t* l, int highlight = -1, float pulse = 0.0f);

} // namespace field
