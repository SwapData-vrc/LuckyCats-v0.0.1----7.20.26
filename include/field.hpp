#pragma once

#ifdef LUCKYCATS_SIM
#include <liblvgl/lvgl.h> // IWYU pragma: keep
#else
#include "liblvgl/lvgl.h" // IWYU pragma: keep
#endif

#include <cstdint>

namespace field {
constexpr int PX = 232;
constexpr float SIZE_IN = 144.0f;
constexpr float HALF_IN = SIZE_IN * 0.5f;
constexpr float PX_PER_IN = PX / SIZE_IN;
constexpr float PI_F = 3.14159265358979f;

inline float px_x(float x_in) { return PX * 0.5f + x_in * PX_PER_IN; }
inline float px_y(float y_in) { return PX * 0.5f - y_in * PX_PER_IN; }
inline float in_x(float p) { return (p - PX * 0.5f) / PX_PER_IN; }
inline float in_y(float p) { return (PX * 0.5f - p) / PX_PER_IN; }

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

void half_disc(lv_layer_t* l, float cx, float cy, float r, int side, uint32_t color,
               lv_opa_t opa = LV_OPA_COVER);
void rot_rect(lv_layer_t* l, float cx, float cy, float w, float h, float sin_t, float cos_t, uint32_t color,
              lv_opa_t opa = LV_OPA_COVER);
}

enum class Alliance : uint8_t { NEUTRAL = 0, RED = 1, BLUE = 2 };

enum class Quad : uint8_t { NORTH = 0, SOUTH = 1, EAST = 2, WEST = 3 };

Quad quad_of(float x, float y);

extern Alliance toggle_owner[4];

void cycle_toggle(Quad q);

enum class GoalKind : uint8_t { SHORT, TALL, ALLIANCE };

struct Goal {
  float x, y;
  float half_w, half_h;
  GoalKind gkind;
  Alliance alliance;
  float min_lift;
};

// !! Field object positions are NOT published machine-readably. Every number in
// field.cpp is measured off the field drawing, inches from centre. Correct them
// there and nothing else needs to change. !!
constexpr int GOAL_COUNT = 9;
extern const Goal goals[GOAL_COUNT];

struct Toggle {
  float x, y;
  Quad quadrant;
  bool horizontal;
};

extern const Toggle toggles[4];

constexpr float TOGGLE_LONG_IN = 7.0f;
constexpr float TOGGLE_THICK_IN = 2.6f;

constexpr float TOGGLE_HIT_IN = 17.0f;

struct Loader {
  float x, y;
  Alliance alliance;
};

extern const Loader loaders[4];

void draw_field_static(lv_layer_t* l);

void draw_toggles(lv_layer_t* l, int highlight = -1, float pulse = 0.0f);
}
