#include "field.hpp" // IWYU pragma: keep

#include <cmath>

namespace field {
namespace col {
constexpr uint32_t TILE_A = 0x55595e;
constexpr uint32_t TILE_B = 0x50545a;
constexpr uint32_t SEAM = 0x474b51;
constexpr uint32_t WALL = 0x20242a;
constexpr uint32_t WALL_HI = 0x333940;
constexpr uint32_t TAPE = 0xeceff2;
constexpr uint32_t RED = 0xd2413f;
constexpr uint32_t BLUE = 0x3d74d0;
constexpr uint32_t YELLOW = 0xe3c341;
constexpr uint32_t GRAY_CUP = 0x9aa1a9;
constexpr uint32_t CLEAR_CUP = 0xd8e2ea;
constexpr uint32_t GOAL_N = 0xb9c2cc;
constexpr uint32_t GOAL_DK = 0x6b737c;
constexpr uint32_t LOADER = 0x2f3742;
constexpr uint32_t SHADOW = 0x000000;
constexpr uint32_t SELECT = 0x4cc9f0;
}

const Goal goals[GOAL_COUNT] = {
    {0.0f, 0.0f, 7.0f, 7.0f, GoalKind::TALL, Alliance::NEUTRAL, 0.75f},

    {0.0f, 40.0f, 8.0f, 6.0f, GoalKind::SHORT, Alliance::NEUTRAL, 0.30f},
    {0.0f, -40.0f, 8.0f, 6.0f, GoalKind::SHORT, Alliance::NEUTRAL, 0.30f},
    {40.0f, 0.0f, 6.0f, 8.0f, GoalKind::SHORT, Alliance::NEUTRAL, 0.30f},
    {-40.0f, 0.0f, 6.0f, 8.0f, GoalKind::SHORT, Alliance::NEUTRAL, 0.30f},

    {-58.0f, 30.0f, 5.0f, 9.0f, GoalKind::ALLIANCE, Alliance::RED, 0.45f},
    {-58.0f, -30.0f, 5.0f, 9.0f, GoalKind::ALLIANCE, Alliance::RED, 0.45f},

    {58.0f, 30.0f, 5.0f, 9.0f, GoalKind::ALLIANCE, Alliance::BLUE, 0.45f},
    {58.0f, -30.0f, 5.0f, 9.0f, GoalKind::ALLIANCE, Alliance::BLUE, 0.45f},
};

const Toggle toggles[4] = {
    {0.0f, 68.0f, Quad::NORTH, true},
    {0.0f, -68.0f, Quad::SOUTH, true},
    {68.0f, 0.0f, Quad::EAST, false},
    {-68.0f, 0.0f, Quad::WEST, false},
};

const Loader loaders[4] = {
    {-66.0f, 54.0f, Alliance::RED},
    {-66.0f, -54.0f, Alliance::RED},
    {66.0f, 54.0f, Alliance::BLUE},
    {66.0f, -54.0f, Alliance::BLUE},
};

Alliance toggle_owner[4] = {Alliance::NEUTRAL, Alliance::NEUTRAL, Alliance::NEUTRAL, Alliance::NEUTRAL};

Quad quad_of(float x, float y) {
  if (y >= std::fabs(x)) return Quad::NORTH;
  if (y <= -std::fabs(x)) return Quad::SOUTH;
  return (x > 0) ? Quad::EAST : Quad::WEST;
}

void cycle_toggle(Quad q) {
  Alliance& a = toggle_owner[static_cast<int>(q)];
  a = (a == Alliance::NEUTRAL) ? Alliance::RED : (a == Alliance::RED ? Alliance::BLUE : Alliance::NEUTRAL);
}

namespace draw {
void line(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, int width, lv_opa_t opa,
          int dash_w, int dash_gap) {
  lv_draw_line_dsc_t d;
  lv_draw_line_dsc_init(&d);
  d.color = lv_color_hex(color);
  d.width = width;
  d.opa = opa;
  d.dash_width = dash_w;
  d.dash_gap = dash_gap;
  d.round_start = 1;
  d.round_end = 1;
  d.p1.x = static_cast<decltype(d.p1.x)>(x1);
  d.p1.y = static_cast<decltype(d.p1.y)>(y1);
  d.p2.x = static_cast<decltype(d.p2.x)>(x2);
  d.p2.y = static_cast<decltype(d.p2.y)>(y2);
  lv_draw_line(l, &d);
}

void rect(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, lv_opa_t opa, int radius) {
  lv_draw_rect_dsc_t d;
  lv_draw_rect_dsc_init(&d);
  d.bg_color = lv_color_hex(color);
  d.bg_opa = opa;
  d.radius = radius;
  d.border_width = 0;
  lv_area_t a{static_cast<int32_t>(x1), static_cast<int32_t>(y1), static_cast<int32_t>(x2),
              static_cast<int32_t>(y2)};
  lv_draw_rect(l, &d, &a);
}

void rect_outline(lv_layer_t* l, float x1, float y1, float x2, float y2, uint32_t color, int width,
                  lv_opa_t opa, int radius) {
  lv_draw_rect_dsc_t d;
  lv_draw_rect_dsc_init(&d);
  d.bg_opa = LV_OPA_TRANSP;
  d.border_color = lv_color_hex(color);
  d.border_width = width;
  d.border_opa = opa;
  d.radius = radius;
  lv_area_t a{static_cast<int32_t>(x1), static_cast<int32_t>(y1), static_cast<int32_t>(x2),
              static_cast<int32_t>(y2)};
  lv_draw_rect(l, &d, &a);
}

void tri(lv_layer_t* l, float ax, float ay, float bx, float by, float cx, float cy, uint32_t color,
         lv_opa_t opa) {
  lv_draw_triangle_dsc_t d;
  lv_draw_triangle_dsc_init(&d);
  d.bg_color = lv_color_hex(color);
  d.bg_opa = opa;
  d.p[0].x = static_cast<decltype(d.p[0].x)>(ax);
  d.p[0].y = static_cast<decltype(d.p[0].y)>(ay);
  d.p[1].x = static_cast<decltype(d.p[1].x)>(bx);
  d.p[1].y = static_cast<decltype(d.p[1].y)>(by);
  d.p[2].x = static_cast<decltype(d.p[2].x)>(cx);
  d.p[2].y = static_cast<decltype(d.p[2].y)>(cy);
  lv_draw_triangle(l, &d);
}

void disc(lv_layer_t* l, float cx, float cy, float r, uint32_t color, lv_opa_t opa) {
  rect(l, cx - r, cy - r, cx + r, cy + r, color, opa, static_cast<int>(r) + 1);
}

void half_disc(lv_layer_t* l, float cx, float cy, float r, int side, uint32_t color, lv_opa_t opa) {
  if (side < 0) rect(l, cx - r, cy - r, cx, cy + r, color, opa, static_cast<int>(r) + 1);
  else rect(l, cx, cy - r, cx + r, cy + r, color, opa, static_cast<int>(r) + 1);
}

void rot_rect(lv_layer_t* l, float cx, float cy, float w, float h, float s, float c, uint32_t color,
              lv_opa_t opa) {
  const float fx = s, fy = -c;
  const float rx = c, ry = s;
  const float hw = w * 0.5f, hh = h * 0.5f;
  const float flx = cx + fx * hh - rx * hw, fly = cy + fy * hh - ry * hw;
  const float frx = cx + fx * hh + rx * hw, fry = cy + fy * hh + ry * hw;
  const float brx = cx - fx * hh + rx * hw, bry = cy - fy * hh + ry * hw;
  const float blx = cx - fx * hh - rx * hw, bly = cy - fy * hh - ry * hw;
  tri(l, flx, fly, frx, fry, brx, bry, color, opa);
  tri(l, flx, fly, brx, bry, blx, bly, color, opa);
}
}

namespace {
void draw_tiles(lv_layer_t* l) {
  constexpr float F = static_cast<float>(PX);
  constexpr float t = F / 6.0f;
  draw::rect(l, 0, 0, F - 1, F - 1, col::TILE_A);
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      if ((r + c) & 1) draw::rect(l, c * t, r * t, (c + 1) * t - 1, (r + 1) * t - 1, col::TILE_B);
  for (int i = 1; i < 6; ++i) {
    draw::line(l, i * t, 0, i * t, F, col::SEAM, 1, LV_OPA_60);
    draw::line(l, 0, i * t, F, i * t, col::SEAM, 1, LV_OPA_60);
  }
}

void draw_quadrants(lv_layer_t* l) {
  constexpr float F = static_cast<float>(PX);
  draw::line(l, 0, 0, F, F, col::TAPE, 2, LV_OPA_70);
  draw::line(l, F, 0, 0, F, col::TAPE, 2, LV_OPA_70);
}

void draw_loaders(lv_layer_t* l) {
  for (const Loader& ld : loaders) {
    const uint32_t a = (ld.alliance == Alliance::RED) ? col::RED : col::BLUE;
    const float w = 4.0f * PX_PER_IN, h = 9.0f * PX_PER_IN;
    const float x = px_x(ld.x), y = px_y(ld.y);
    draw::rect(l, x - w, y - h, x + w, y + h, col::LOADER, LV_OPA_COVER, 3);
    draw::rect_outline(l, x - w, y - h, x + w, y + h, a, 2, LV_OPA_90, 3);
  }
}

void draw_goals_static(lv_layer_t* l) {
  for (const Goal& g : goals) {
    const float x1 = px_x(g.x - g.half_w), x2 = px_x(g.x + g.half_w);
    const float y1 = px_y(g.y + g.half_h), y2 = px_y(g.y - g.half_h);

    uint32_t body = col::GOAL_N;
    if (g.alliance == Alliance::RED) body = col::RED;
    else if (g.alliance == Alliance::BLUE) body = col::BLUE;

    draw::rect(l, x1 + 1.5f, y1 + 1.5f, x2 + 1.5f, y2 + 1.5f, col::SHADOW, LV_OPA_30, 3);
    draw::rect(l, x1, y1, x2, y2, body, LV_OPA_COVER, 3);
    draw::rect_outline(l, x1, y1, x2, y2, col::GOAL_DK, 1, LV_OPA_COVER, 3);

    if (g.gkind == GoalKind::TALL) {
      draw::rect(l, x1 + 3, y1 + 3, x2 - 3, y2 - 3, col::GOAL_DK, LV_OPA_COVER, 2);
      draw::rect_outline(l, x1 - 3, y1 - 3, x2 + 3, y2 + 3, col::TAPE, 1, LV_OPA_40, 4);
    }
  }
}

void draw_wall(lv_layer_t* l) {
  draw::rect_outline(l, 0, 0, PX - 1, PX - 1, col::WALL, 4, LV_OPA_COVER, 3);
  draw::rect_outline(l, 2, 2, PX - 3, PX - 3, col::WALL_HI, 1, LV_OPA_60, 2);
}
}

void draw_field_static(lv_layer_t* l) {
  draw_tiles(l);
  draw_quadrants(l);
  draw_loaders(l);
  draw_goals_static(l);
  draw_wall(l);
}

namespace {
void draw_quadrant_wash(lv_layer_t* l) {
  constexpr float F = static_cast<float>(PX);
  constexpr float C = F * 0.5f;

  for (int q = 0; q < 4; ++q) {
    const Alliance a = toggle_owner[q];
    if (a == Alliance::NEUTRAL) continue;
    const uint32_t c = (a == Alliance::RED) ? col::RED : col::BLUE;

    switch (static_cast<Quad>(q)) {
      case Quad::NORTH: draw::tri(l, C, C, 0, 0, F, 0, c, LV_OPA_20); break;
      case Quad::SOUTH: draw::tri(l, C, C, 0, F, F, F, c, LV_OPA_20); break;
      case Quad::EAST: draw::tri(l, C, C, F, 0, F, F, c, LV_OPA_20); break;
      case Quad::WEST: draw::tri(l, C, C, 0, 0, 0, F, c, LV_OPA_20); break;
    }
  }
}
}

void draw_toggles(lv_layer_t* l, int highlight, float pulse) {
  draw_quadrant_wash(l);

  for (const Toggle& t : toggles) {
    const int qi = static_cast<int>(t.quadrant);
    const Alliance a = toggle_owner[qi];
    uint32_t c = col::YELLOW;
    if (a == Alliance::RED) c = col::RED;
    else if (a == Alliance::BLUE) c = col::BLUE;

    const float x = px_x(t.x), y = px_y(t.y);
    const float lng = TOGGLE_LONG_IN * PX_PER_IN, thk = TOGGLE_THICK_IN * PX_PER_IN;
    const float w = t.horizontal ? lng : thk;
    const float h = t.horizontal ? thk : lng;

    if (qi == highlight) {
      const float grow = 3.0f + 4.0f * pulse;
      draw::rect(l, x - w - grow, y - h - grow, x + w + grow, y + h + grow, col::SELECT, LV_OPA_30, 6);
      draw::rect_outline(l, x - w - grow, y - h - grow, x + w + grow, y + h + grow, col::SELECT, 2,
                         LV_OPA_80, 6);
    }

    draw::rect(l, x - w, y - h, x + w, y + h, 0x14181d, LV_OPA_COVER, 3);
    draw::rect(l, x - w + 1.5f, y - h + 1.5f, x + w - 1.5f, y + h - 1.5f, c, LV_OPA_COVER, 3);

    for (int r = -1; r <= 1; ++r) {
      const float f = static_cast<float>(r) * 0.45f;
      if (t.horizontal) draw::line(l, x + w * f, y - h + 2, x + w * f, y + h - 2, 0x14181d, 1, LV_OPA_50);
      else draw::line(l, x - w + 2, y + h * f, x + w - 2, y + h * f, 0x14181d, 1, LV_OPA_50);
    }
  }
}
}
