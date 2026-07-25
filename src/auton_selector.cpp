#include "auton_selector.hpp" // IWYU pragma: keep
#include "field.hpp"          // IWYU pragma: keep

#include <cmath>
#include <cstdio>
#include <cstring>

// The field art, baked in from src/field_img.c (232x232 ARGB8888). This must
// sit at global scope: inside the anonymous namespace below it would get
// internal linkage and fail to bind to the C symbol.
LV_IMAGE_DECLARE(field_img);

// ---- TEAM LOGO -------------------------------------------------------------
// The team badge, baked in from src/logo_img.c (113x148 ARGB8888).
//
// lv_image does NOT scale to fit -- it sizes itself to the source and overflows
// its parent. Artwork must therefore arrive pre-fitted to LOGO_BOX (see
// build_landing). Comment out both lines below to fall back to the drawn
// placeholder badge.
LV_IMAGE_DECLARE(logo_img);
#define HAVE_LOGO_IMAGE 1
// ----------------------------------------------------------------------------

namespace auton {
namespace {

using field::PX_PER_IN;
using field::px_x;
using field::px_y;

// ---------------------------------------------------------------------------
// Layout. PROS gives LVGL 480x240 -- the V5 keeps the top strip for itself.
// ---------------------------------------------------------------------------

constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240;

constexpr int CARD_X = 8;
constexpr int CARD_Y = 4;
constexpr int CARD_W = 224;
constexpr int CARD_H = 232;
constexpr int CARD_PAD = 10;
constexpr int COL_W = CARD_W - 2 * CARD_PAD; // 204
constexpr int COL_H = CARD_H - 2 * CARD_PAD; // 212

constexpr int FIELD_X = 240;
constexpr int FIELD_Y = 4;

constexpr int ROW_H = 30; // dropdowns
constexpr int BTN_H = 28;

constexpr float ROBOT_IN = 15.0f;
constexpr float PI_F = field::PI_F;

namespace ink {
constexpr uint32_t BG = 0x0b0e13;
constexpr uint32_t CARD = 0x151a21;
constexpr uint32_t EDGE = 0x232a33;
constexpr uint32_t CTRL = 0x1d232b;
constexpr uint32_t CTRL_HI = 0x27303a;
constexpr uint32_t SUNK = 0x0e1218;
constexpr uint32_t TEXT = 0xe6edf3;
constexpr uint32_t DIM = 0x7d8590;
constexpr uint32_t ACCENT = 0x4cc9f0;
constexpr uint32_t GOOD = 0x3fb950;
constexpr uint32_t WARN = 0xd29922;
constexpr uint32_t RED = 0xda3633;
constexpr uint32_t BLUE = 0x388bfd;
} // namespace ink

uint32_t alliance_ink(field::Alliance a) { return a == field::Alliance::BLUE ? ink::BLUE : ink::RED; }

// ---------------------------------------------------------------------------
// Route steps
// ---------------------------------------------------------------------------

enum class Kind : uint8_t {
  DRIVE,  // a = inches along current heading, negative reverses
  TURN,   // a = absolute heading, degrees
  GOTO,   // a,b = field point in inches; flag bit 0 = swerve
  INTAKE, // a = +1 in, -1 out, 0 stop
  CLAW,   // a = 1 close/grip, 0 open/release
  LIFT,   // a = 0..1 target height
  WAIT,   // a = milliseconds
};

/// GOTO modifier: without it the robot turns to the bearing first and then
/// drives straight; with it the robot arcs to the point in one motion.
constexpr uint8_t F_SWERVE = 1;

struct Step {
  Kind kind;
  float a, b;
  uint8_t flag;
};

constexpr int MAX_STEPS = 20;

struct RouteBuf {
  Step s[MAX_STEPS];
  int n;
};

// ---------------------------------------------------------------------------
// Starting positions
//
// Authored in the RED frame: the red Alliance Station is the -X wall, so a red
// robot starts against it facing +X (heading 90). BLUE mirrors x and heading.
// ---------------------------------------------------------------------------

struct Start {
  float x, y, th;
};

const Start STARTS[] = {
    {0.0f, 0.0f, 0.0f}, // index 0 means "use the route's own default"
    {-52.0f, 44.0f, 90.0f},
    {-52.0f, 0.0f, 90.0f},
    {-52.0f, -44.0f, 90.0f},
    {0.0f, 0.0f, 0.0f},
};
constexpr int START_COUNT = 5;

const char* const START_OPTIONS = "Route default\nLoader +Y\nWall mid\nLoader -Y\nField centre";

// ---------------------------------------------------------------------------
// Presets, in the RED frame
// ---------------------------------------------------------------------------

constexpr Step P_FWD[] = {{Kind::DRIVE, 10.0f, 0, 0}};
constexpr Step P_BACK[] = {{Kind::DRIVE, -10.0f, 0, 0}};
constexpr Step P_TURN[] = {{Kind::TURN, 90.0f, 0, 0}};

struct Preset {
  const char* name;
  const Step* s;
  int n;
  float sx, sy, sth; // start pose used when the start dropdown says "default"
};

const Preset PRESETS[3] = {
    {"Forward 10 in", P_FWD, 1, -52.0f, 0.0f, 90.0f},
    {"Backward 10 in", P_BACK, 1, -40.0f, 0.0f, 90.0f},
    {"Turn 90 CW", P_TURN, 1, -52.0f, 0.0f, 90.0f},
};

const char* const ROUTE_OPTIONS = "Forward 10 in\nBackward 10 in\nTurn 90 CW\nCustom route";

// The custom route. RAM only -- never written to flash, so it is gone on power
// cycle. It is kept after a run so you can re-run the same test.
RouteBuf g_custom{{}, 0};
float g_custom_start[3] = {-52.0f, 0.0f, 90.0f};

Route g_selected = Route::FORWARD_10;
field::Alliance g_alliance = field::Alliance::RED;
int g_start_sel = 0;

int step_count() {
  return (g_selected == Route::CUSTOM) ? g_custom.n : PRESETS[static_cast<int>(g_selected)].n;
}

const Step* raw_step_at(int i) {
  return (g_selected == Route::CUSTOM) ? &g_custom.s[i] : &PRESETS[static_cast<int>(g_selected)].s[i];
}

bool mirrored() { return g_alliance == field::Alliance::BLUE; }

/// A step as it will actually be driven, after alliance mirroring.
Step step_at(int i) {
  Step s = *raw_step_at(i);
  if (mirrored()) {
    // mirror across the Y axis: x negates, and a clockwise heading becomes
    // the same magnitude counter-clockwise
    if (s.kind == Kind::TURN) s.a = -s.a;
    else if (s.kind == Kind::GOTO) s.a = -s.a;
  }
  return s;
}

void start_pose(float& x, float& y, float& th) {
  if (g_start_sel == 0) {
    if (g_selected == Route::CUSTOM) {
      x = g_custom_start[0];
      y = g_custom_start[1];
      th = g_custom_start[2];
    } else {
      const Preset& p = PRESETS[static_cast<int>(g_selected)];
      x = p.sx;
      y = p.sy;
      th = p.sth;
    }
  } else {
    const Start& s = STARTS[g_start_sel];
    x = s.x;
    y = s.y;
    th = s.th;
  }
  if (mirrored()) {
    x = -x;
    th = -th;
  }
}

/// Bearing from (x,y) to (tx,ty). LemLib heading: 0 faces +Y, clockwise up.
float bearing_to(float x, float y, float tx, float ty) {
  const float dx = tx - x, dy = ty - y;
  if (std::fabs(dx) < 0.01f && std::fabs(dy) < 0.01f) return 0.0f;
  return std::atan2(dx, dy) * 180.0f / PI_F;
}

void step_text(const Step& s, char* out, int n) {
  switch (s.kind) {
    case Kind::DRIVE: std::snprintf(out, n, "Drive   %.0f in", static_cast<double>(s.a)); break;
    case Kind::TURN: std::snprintf(out, n, "Turn    %.0f deg", static_cast<double>(s.a)); break;
    case Kind::GOTO:
      std::snprintf(out, n, "%s  %.0f, %.0f", (s.flag & F_SWERVE) ? "Swerve" : "Goto  ",
                    static_cast<double>(s.a), static_cast<double>(s.b));
      break;
    case Kind::INTAKE:
      std::snprintf(out, n, "Intake  %s", s.a > 0 ? "IN" : (s.a < 0 ? "OUT" : "STOP"));
      break;
    case Kind::CLAW: std::snprintf(out, n, "Claw    %s", s.a > 0.5f ? "GRIP" : "RELEASE"); break;
    case Kind::LIFT: std::snprintf(out, n, "Lift    %.0f%%", static_cast<double>(s.a * 100.0f)); break;
    case Kind::WAIT: std::snprintf(out, n, "Wait    %.0f ms", static_cast<double>(s.a)); break;
  }
}

// ---------------------------------------------------------------------------
// Preview simulation
//
// Runs forward frame by frame rather than being evaluated at an arbitrary t,
// because toggle flips are stateful.
// ---------------------------------------------------------------------------

constexpr uint32_t FRAME_MS = 50; // 20 fps
constexpr uint32_t END_HOLD_MS = 900;

constexpr float REACH_IN = 4.0f;   // intake sits this far in front of the bumper
constexpr float REACH_R_IN = 5.0f; // radius the intake acts over

struct Sim {
  int step_i;
  int phase; // GOTO only: 0 = turning to bearing, 1 = driving
  uint32_t t_in_step;
  uint32_t hold;
  float x, y, th;    // live pose
  float sx, sy, sth; // pose when the current phase began
  float tx, ty, tth; // pose the current phase is heading for
  float lift;        // 0..1
  float lift_from, lift_to;
  int intake; // -1 out, 0 stop, +1 in
  bool claw;  // true = closed / gripping
  bool finished;
};

Sim g_sim{};
bool g_robot_selected = false;

uint32_t phase_ms(const Step& s, int phase) {
  switch (s.kind) {
    case Kind::DRIVE: return static_cast<uint32_t>(std::fabs(s.a) * 45.0f) + 250;
    case Kind::TURN: return 700;
    case Kind::GOTO:
      if (s.flag & F_SWERVE) return 1100;
      return (phase == 0) ? 500 : 900;
    case Kind::INTAKE: return 500;
    case Kind::CLAW: return 400;
    case Kind::LIFT: return 700;
    case Kind::WAIT: return static_cast<uint32_t>(s.a);
  }
  return 300;
}

/// Where the intake mouth sits, in field inches.
void claw_point(float x, float y, float th, float& cx, float& cy) {
  const float t = th * PI_F / 180.0f;
  const float reach = ROBOT_IN * 0.5f + REACH_IN;
  cx = x + std::sin(t) * reach;
  cy = y + std::cos(t) * reach;
}

/// Spin the toggle the robot is parked against, if the intake is running.
void try_toggle() {
  float cx, cy;
  claw_point(g_sim.x, g_sim.y, g_sim.th, cx, cy);
  for (const field::Toggle& t : field::toggles) {
    const float dx = t.x - cx, dy = t.y - cy;
    if (dx * dx + dy * dy > 36.0f) continue;
    field::toggle_owner[static_cast<int>(t.quadrant)] = g_alliance;
    return;
  }
}

void set_leg(float tx, float ty, float tth) {
  g_sim.t_in_step = 0;
  g_sim.sx = g_sim.x;
  g_sim.sy = g_sim.y;
  g_sim.sth = g_sim.th;
  g_sim.tx = tx;
  g_sim.ty = ty;
  g_sim.tth = tth;
  g_sim.lift_from = g_sim.lift;
  g_sim.lift_to = g_sim.lift;
}

void begin_step() {
  g_sim.phase = 0;
  set_leg(g_sim.x, g_sim.y, g_sim.th);

  if (g_sim.step_i >= step_count()) return;
  const Step s = step_at(g_sim.step_i);

  switch (s.kind) {
    case Kind::DRIVE: {
      const float t = g_sim.th * PI_F / 180.0f;
      set_leg(g_sim.x + std::sin(t) * s.a, g_sim.y + std::cos(t) * s.a, g_sim.th);
      break;
    }
    case Kind::TURN:
      set_leg(g_sim.x, g_sim.y, s.a);
      break;
    case Kind::GOTO: {
      const float bear = bearing_to(g_sim.x, g_sim.y, s.a, s.b);
      if (s.flag & F_SWERVE) {
        // one continuous arc: position and heading resolve together
        g_sim.phase = 1;
        set_leg(s.a, s.b, bear);
      } else {
        // phase 0 turns in place; phase 1 drives the straight line
        set_leg(g_sim.x, g_sim.y, bear);
      }
      break;
    }
    case Kind::INTAKE:
      g_sim.intake = static_cast<int>(s.a);
      break;
    case Kind::CLAW:
      g_sim.claw = (s.a > 0.5f);
      break;
    case Kind::LIFT:
      g_sim.lift_to = s.a;
      break;
    case Kind::WAIT:
      break;
  }
}

void sim_reset() {
  for (int i = 0; i < 4; ++i) field::toggle_owner[i] = field::Alliance::NEUTRAL;
  g_sim = Sim{};
  start_pose(g_sim.x, g_sim.y, g_sim.th);
  begin_step();
}

float ease(float u) { return u * u * (3.0f - 2.0f * u); }

void sim_tick() {
  if (g_sim.finished) {
    if (g_sim.hold > END_HOLD_MS) sim_reset();
    else g_sim.hold += FRAME_MS;
    return;
  }
  if (g_sim.step_i >= step_count()) {
    g_sim.finished = true;
    g_sim.hold = 0;
    return;
  }

  const Step s = step_at(g_sim.step_i);
  const uint32_t dur = phase_ms(s, g_sim.phase) ? phase_ms(s, g_sim.phase) : 1;

  g_sim.t_in_step += FRAME_MS;
  const float raw = static_cast<float>(g_sim.t_in_step) / static_cast<float>(dur);
  const float u = ease(raw > 1.0f ? 1.0f : raw);

  g_sim.x = g_sim.sx + (g_sim.tx - g_sim.sx) * u;
  g_sim.y = g_sim.sy + (g_sim.ty - g_sim.sy) * u;
  g_sim.th = g_sim.sth + (g_sim.tth - g_sim.sth) * u;
  g_sim.lift = g_sim.lift_from + (g_sim.lift_to - g_sim.lift_from) * u;

  if (g_sim.intake != 0) try_toggle();

  if (raw < 1.0f) return;

  g_sim.x = g_sim.tx;
  g_sim.y = g_sim.ty;
  g_sim.th = g_sim.tth;
  g_sim.lift = g_sim.lift_to;

  // a non-swerve GOTO has finished turning; now drive the straight leg
  if (s.kind == Kind::GOTO && g_sim.phase == 0) {
    g_sim.phase = 1;
    set_leg(s.a, s.b, g_sim.th);
    return;
  }

  ++g_sim.step_i;
  if (g_sim.step_i >= step_count()) {
    g_sim.finished = true;
    g_sim.hold = 0;
  } else {
    begin_step();
  }
}

// ---------------------------------------------------------------------------
// Live telemetry and route recording
// ---------------------------------------------------------------------------

constexpr int TRAIL_MAX = 160;
constexpr float TRAIL_MIN_IN = 1.2f;  // breadcrumb spacing
constexpr float REC_MIN_IN = 8.0f;    // recorded waypoint spacing

struct Pt {
  float x, y;
};

Pt g_trail[TRAIL_MAX];
int g_trail_n = 0;

Pt g_rec[MAX_STEPS];
int g_rec_n = 0;
float g_rec_start[3] = {0, 0, 0};

float g_live_x = 0, g_live_y = 0, g_live_th = 0;

bool g_recording = false;
volatile bool g_req_live = false;
volatile bool g_req_rec_toggle = false;

void trail_push(float x, float y) {
  if (g_trail_n > 0) {
    const float dx = x - g_trail[g_trail_n - 1].x, dy = y - g_trail[g_trail_n - 1].y;
    if (dx * dx + dy * dy < TRAIL_MIN_IN * TRAIL_MIN_IN) return;
  }
  if (g_trail_n >= TRAIL_MAX) {
    // ring down: drop the oldest half so the recent path stays visible
    std::memmove(g_trail, g_trail + TRAIL_MAX / 2, sizeof(Pt) * (TRAIL_MAX / 2));
    g_trail_n = TRAIL_MAX / 2;
  }
  g_trail[g_trail_n++] = Pt{x, y};
}

/// Turn the recorded breadcrumbs into a custom route of GOTO steps.
void recording_commit() {
  g_custom.n = 0;
  // store in the RED frame so the blue mirror reproduces what was driven
  const float sgn = mirrored() ? -1.0f : 1.0f;
  g_custom_start[0] = g_rec_start[0] * sgn;
  g_custom_start[1] = g_rec_start[1];
  g_custom_start[2] = g_rec_start[2] * sgn;

  for (int i = 0; i < g_rec_n && g_custom.n < MAX_STEPS; ++i)
    g_custom.s[g_custom.n++] = Step{Kind::GOTO, g_rec[i].x * sgn, g_rec[i].y, 0};

  g_selected = Route::CUSTOM;
}

void live_sample() {
  const lemlib::Pose p = chassis.getPose();
  g_live_x = static_cast<float>(p.x);
  g_live_y = static_cast<float>(p.y);
  g_live_th = static_cast<float>(p.theta);

  trail_push(g_live_x, g_live_y);

  if (!g_recording) return;
  if (g_rec_n == 0) {
    g_rec_start[0] = g_live_x;
    g_rec_start[1] = g_live_y;
    g_rec_start[2] = g_live_th;
    g_rec[g_rec_n++] = Pt{g_live_x, g_live_y};
    return;
  }
  const float dx = g_live_x - g_rec[g_rec_n - 1].x, dy = g_live_y - g_rec[g_rec_n - 1].y;
  if (dx * dx + dy * dy < REC_MIN_IN * REC_MIN_IN) return;
  if (g_rec_n < MAX_STEPS) g_rec[g_rec_n++] = Pt{g_live_x, g_live_y};
}

// ---------------------------------------------------------------------------
// Robot rendering
// ---------------------------------------------------------------------------

/// ghost = translucent outline only. mech = draw lift bar / claw / intake ring,
/// which only the simulated robot has state for.
void draw_chassis(lv_layer_t* l, float x, float y, float th, bool ghost, bool mech) {
  const float t = th * PI_F / 180.0f;
  const float s = std::sin(t), c = std::cos(t);
  const float fx = s, fy = -c; // forward, screen space
  const float rx = c, ry = s;  // right, screen space

  const float cx = px_x(x);
  const float cy = px_y(y);
  const float body = ROBOT_IN * PX_PER_IN;
  const float half = body * 0.5f;

  if (ghost) {
    field::draw::rot_rect(l, cx, cy, body, body, s, c, ink::ACCENT, LV_OPA_20);
    field::draw::tri(l, cx + fx * half - rx * half * 0.30f, cy + fy * half - ry * half * 0.30f,
                     cx + fx * half + rx * half * 0.30f, cy + fy * half + ry * half * 0.30f,
                     cx + fx * half * 1.30f, cy + fy * half * 1.30f, ink::ACCENT, LV_OPA_50);
    return;
  }

  // shadow
  field::draw::rot_rect(l, cx + 2.0f, cy + 2.0f, body, body, s, c, 0x000000, LV_OPA_30);

  // wheels, before the body so they read as underneath
  for (int i = 0; i < 4; ++i) {
    const float ox = (i & 1) ? half * 0.92f : -half * 0.92f;
    const float oy = (i & 2) ? half * 0.55f : -half * 0.55f;
    field::draw::rot_rect(l, cx + rx * ox + fx * oy, cy + ry * ox + fy * oy, body * 0.13f, body * 0.30f, s, c,
                          0x15181c, LV_OPA_COVER);
  }

  // chassis, in the selected alliance's colour so the mirror is obvious
  field::draw::rot_rect(l, cx, cy, body, body, s, c, 0x2f3742, LV_OPA_COVER);
  field::draw::rot_rect(l, cx + fx * half * 0.10f, cy + fy * half * 0.10f, body * 0.74f, body * 0.62f, s, c,
                        alliance_ink(g_alliance), LV_OPA_70);

  if (mech) {
    // lift indicator: fills from the back of the deck forward as the lift rises
    const float bw = body * 0.52f;
    const float bh = body * 0.13f;
    const float bx = cx - fx * half * 0.42f;
    const float by = cy - fy * half * 0.42f;
    field::draw::rot_rect(l, bx, by, bw, bh, s, c, 0x11151a, LV_OPA_COVER);
    if (g_sim.lift > 0.01f) {
      const float fwv = bw * g_sim.lift;
      field::draw::rot_rect(l, bx - rx * (bw - fwv) * 0.5f, by - ry * (bw - fwv) * 0.5f, fwv, bh * 0.72f, s, c,
                            ink::ACCENT, LV_OPA_COVER);
    }

    // claw prongs at the front, spread when open
    const float spread = g_sim.claw ? 0.20f : 0.42f;
    const float len = body * 0.30f;
    const float pxp = cx + fx * half, pyp = cy + fy * half;
    for (int side = -1; side <= 1; side += 2) {
      const float ox = rx * body * spread * side;
      const float oy = ry * body * spread * side;
      field::draw::rot_rect(l, pxp + ox + fx * len * 0.5f, pyp + oy + fy * len * 0.5f, body * 0.10f, len, s, c,
                            0xb9c2cc, LV_OPA_COVER);
    }

    // intake state ring: green pulling in, orange pushing out
    if (g_sim.intake != 0) {
      const uint32_t ic = (g_sim.intake > 0) ? ink::GOOD : ink::WARN;
      float gx, gy;
      claw_point(x, y, th, gx, gy);
      field::draw::disc(l, px_x(gx), px_y(gy), REACH_R_IN * PX_PER_IN, ic, LV_OPA_20);
      field::draw::disc(l, px_x(gx), px_y(gy), 2.2f, ic, LV_OPA_COVER);
    }
  }

  // heading nose
  const float nx = cx + fx * half * 1.35f, ny = cy + fy * half * 1.35f;
  field::draw::tri(l, cx + fx * half - rx * half * 0.30f, cy + fy * half - ry * half * 0.30f,
                   cx + fx * half + rx * half * 0.30f, cy + fy * half + ry * half * 0.30f, nx, ny, 0xffffff,
                   LV_OPA_90);

  // selection ring when the driver has tapped the robot
  if (g_robot_selected) {
    const float r = half * 1.6f;
    field::draw::rect_outline(l, cx - r, cy - r, cx + r, cy + r, ink::ACCENT, 2, LV_OPA_90,
                              static_cast<int>(r));
  }
}

// ---------------------------------------------------------------------------
// Canvas
// ---------------------------------------------------------------------------

lv_obj_t* g_canvas = nullptr;

alignas(64) uint8_t g_canvas_buf[LV_CANVAS_BUF_SIZE(field::PX, field::PX, 32, LV_DRAW_BUF_STRIDE_ALIGN)];
alignas(64) uint8_t g_bg_buf[LV_CANVAS_BUF_SIZE(field::PX, field::PX, 32, LV_DRAW_BUF_STRIDE_ALIGN)];
bool g_bg_ready = false;

// The field art is decoded exactly once, here, into the cached background.
// That matters: LV_CACHE_DEF_SIZE is 0 in this build, so an lv_image widget
// would re-run the decoder on every invalidate -- 20 times a second.
const void* const FIELD_IMAGE_SRC = &field_img;

bool draw_field_image(lv_layer_t* l) {
  if (FIELD_IMAGE_SRC == nullptr) return false;

  lv_image_header_t hdr;
  if (lv_image_decoder_get_info(FIELD_IMAGE_SRC, &hdr) != LV_RESULT_OK) return false;
  if (hdr.w != field::PX || hdr.h != field::PX) return false;

  lv_area_t a{0, 0, field::PX - 1, field::PX - 1};
  lv_draw_image_dsc_t d;
  lv_draw_image_dsc_init(&d);
  d.src = FIELD_IMAGE_SRC;
  d.header = hdr;
  d.image_area = a;
  lv_draw_image(l, &d, &a);
  return true;
}

void build_background() {
  // Opaque base first: the art is ARGB8888, so any transparent pixel in it
  // would otherwise composite against whatever the buffer happened to hold.
  lv_canvas_fill_bg(g_canvas, lv_color_hex(ink::BG), LV_OPA_COVER);

  lv_layer_t layer;
  lv_canvas_init_layer(g_canvas, &layer);
  if (!draw_field_image(&layer)) field::draw_field_static(&layer);
  lv_canvas_finish_layer(g_canvas, &layer);
  std::memcpy(g_bg_buf, g_canvas_buf, sizeof(g_bg_buf));
  g_bg_ready = true;
}

void draw_planned_path(lv_layer_t* l, lv_opa_t opa) {
  // walk the steps geometrically to trace where the route goes
  float x, y, th;
  start_pose(x, y, th);

  for (int i = 0; i < step_count(); ++i) {
    const Step s = step_at(i);
    float nx = x, ny = y;
    if (s.kind == Kind::DRIVE) {
      const float t = th * PI_F / 180.0f;
      nx = x + std::sin(t) * s.a;
      ny = y + std::cos(t) * s.a;
    } else if (s.kind == Kind::GOTO) {
      nx = s.a;
      ny = s.b;
      th = bearing_to(x, y, nx, ny);
    } else if (s.kind == Kind::TURN) {
      th = s.a;
    }
    if (nx != x || ny != y) {
      const bool swerve = (s.kind == Kind::GOTO) && (s.flag & F_SWERVE);
      field::draw::line(l, px_x(x), px_y(y), px_x(nx), px_y(ny), ink::ACCENT, 2, opa, swerve ? 0 : 4,
                        swerve ? 0 : 3);
      field::draw::disc(l, px_x(nx), px_y(ny), 2.5f, ink::ACCENT, opa);
    }
    x = nx;
    y = ny;
  }
}

void draw_trail(lv_layer_t* l) {
  const uint32_t c = g_recording ? ink::WARN : ink::GOOD;
  for (int i = 1; i < g_trail_n; ++i)
    field::draw::line(l, px_x(g_trail[i - 1].x), px_y(g_trail[i - 1].y), px_x(g_trail[i].x),
                      px_y(g_trail[i].y), c, 2, LV_OPA_80);

  // recorded waypoints sit on top as solid dots
  if (g_recording)
    for (int i = 0; i < g_rec_n; ++i)
      field::draw::disc(l, px_x(g_rec[i].x), px_y(g_rec[i].y), 3.0f, ink::WARN, LV_OPA_COVER);
}

// ---------------------------------------------------------------------------
// Views
// ---------------------------------------------------------------------------

enum class View : uint8_t { LANDING = 0, SELECT = 1, EDIT = 2, LIVE = 3 };

View g_view = View::LANDING;
lv_obj_t* g_root[4] = {nullptr, nullptr, nullptr, nullptr};

// SELECT widgets
lv_obj_t* g_dd_alliance = nullptr;
lv_obj_t* g_dd_route = nullptr;
lv_obj_t* g_dd_start = nullptr;
lv_obj_t* g_lbl_pose = nullptr;
lv_obj_t* g_lbl_step = nullptr;

// EDIT widgets
lv_obj_t* g_dd_add = nullptr;
lv_obj_t* g_step_row[MAX_STEPS] = {nullptr};
lv_obj_t* g_step_lbl[MAX_STEPS] = {nullptr};
lv_obj_t* g_lbl_empty = nullptr;

// LIVE widgets
lv_obj_t* g_lbl_lx = nullptr;
lv_obj_t* g_lbl_ly = nullptr;
lv_obj_t* g_lbl_lh = nullptr;
lv_obj_t* g_lbl_batt = nullptr;
lv_obj_t* g_lbl_rec = nullptr;
lv_obj_t* g_btn_rec_lbl = nullptr;

const char* route_name(Route r) {
  if (r == Route::CUSTOM) return "Custom route";
  return PRESETS[static_cast<int>(r)].name;
}

bool canvas_visible() { return g_view != View::LANDING; }

void set_view(View v) {
  g_view = v;
  for (int i = 0; i < 4; ++i) {
    if (g_root[i] == nullptr) continue;
    if (i == static_cast<int>(v)) lv_obj_remove_flag(g_root[i], LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(g_root[i], LV_OBJ_FLAG_HIDDEN);
  }
  if (g_canvas != nullptr) {
    if (canvas_visible()) lv_obj_remove_flag(g_canvas, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(g_canvas, LV_OBJ_FLAG_HIDDEN);
  }
}

void refresh_steps() {
  for (int i = 0; i < MAX_STEPS; ++i) {
    if (g_step_row[i] == nullptr) continue;
    if (i < g_custom.n) {
      char b[40], t[52];
      step_text(g_custom.s[i], b, sizeof(b));
      std::snprintf(t, sizeof(t), "%d.  %s", i + 1, b);
      lv_label_set_text(g_step_lbl[i], t);
      lv_obj_remove_flag(g_step_row[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(g_step_row[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
  if (g_lbl_empty != nullptr) {
    if (g_custom.n == 0) lv_obj_remove_flag(g_lbl_empty, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(g_lbl_empty, LV_OBJ_FLAG_HIDDEN);
  }
}

void update_readout() {
  char buf[64];

  if (g_lbl_pose != nullptr) {
    std::snprintf(buf, sizeof(buf), "X %.1f   Y %.1f   H %.0f", static_cast<double>(g_sim.x),
                  static_cast<double>(g_sim.y), static_cast<double>(g_sim.th));
    lv_label_set_text(g_lbl_pose, buf);
    lv_obj_set_style_text_color(g_lbl_pose, lv_color_hex(g_robot_selected ? ink::ACCENT : ink::TEXT),
                                LV_PART_MAIN);
  }

  if (g_lbl_step != nullptr) {
    const int n = step_count();
    if (g_sim.finished || n == 0) {
      std::snprintf(buf, sizeof(buf), "%s  -  %d step%s", route_name(g_selected), n, n == 1 ? "" : "s");
    } else {
      char b[40];
      step_text(step_at(g_sim.step_i), b, sizeof(b));
      std::snprintf(buf, sizeof(buf), "%d/%d  %s", g_sim.step_i + 1, n, b);
    }
    lv_label_set_text(g_lbl_step, buf);
  }
}

void update_live_readout() {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(g_live_x));
  lv_label_set_text(g_lbl_lx, buf);
  std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(g_live_y));
  lv_label_set_text(g_lbl_ly, buf);
  std::snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(g_live_th));
  lv_label_set_text(g_lbl_lh, buf);

  const double cap = pros::battery::get_capacity();
  std::snprintf(buf, sizeof(buf), "%.0f%%", cap);
  lv_label_set_text(g_lbl_batt, buf);
  lv_obj_set_style_text_color(g_lbl_batt, lv_color_hex(cap < 30.0 ? ink::WARN : ink::GOOD), LV_PART_MAIN);

  if (g_recording) {
    std::snprintf(buf, sizeof(buf), "REC  %d pt", g_rec_n);
    lv_label_set_text(g_lbl_rec, buf);
    lv_obj_set_style_text_color(g_lbl_rec, lv_color_hex(ink::WARN), LV_PART_MAIN);
    lv_label_set_text(g_btn_rec_lbl, "Stop recording");
  } else {
    lv_label_set_text(g_lbl_rec, "idle");
    lv_obj_set_style_text_color(g_lbl_rec, lv_color_hex(ink::DIM), LV_PART_MAIN);
    lv_label_set_text(g_btn_rec_lbl, "Record route");
  }
}

void redraw() {
  if (g_canvas == nullptr || !canvas_visible()) return;
  if (!g_bg_ready) build_background();

  std::memcpy(g_canvas_buf, g_bg_buf, sizeof(g_canvas_buf));

  lv_layer_t layer;
  lv_canvas_init_layer(g_canvas, &layer);
  field::draw_toggles(&layer);

  if (g_view == View::LIVE) {
    draw_planned_path(&layer, LV_OPA_30); // dim, for comparison
    draw_trail(&layer);
    draw_chassis(&layer, g_live_x, g_live_y, g_live_th, false, false);
  } else {
    draw_planned_path(&layer, LV_OPA_70);
    float sx, sy, sth;
    start_pose(sx, sy, sth);
    draw_chassis(&layer, sx, sy, sth, true, false); // ghost at the start pose
    draw_chassis(&layer, g_sim.x, g_sim.y, g_sim.th, false, true);
  }

  lv_canvas_finish_layer(g_canvas, &layer);
  lv_obj_invalidate(g_canvas);

  if (g_view == View::LIVE) update_live_readout();
  else update_readout();
}

void restart_preview() {
  refresh_steps();
  sim_reset();
  redraw();
}

void push_step(Kind k, float a, float b = 0, uint8_t flag = 0) {
  if (g_custom.n >= MAX_STEPS) return;
  g_custom.s[g_custom.n++] = Step{k, a, b, flag};
  restart_preview();
}

// ---------------------------------------------------------------------------
// Events
// ---------------------------------------------------------------------------

void nav_cb(lv_event_t* e) {
  const int tag = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
  set_view(static_cast<View>(tag));
  if (g_view == View::SELECT || g_view == View::EDIT) restart_preview();
  else redraw();
}

void alliance_cb(lv_event_t* e) {
  g_alliance = (lv_dropdown_get_selected(lv_event_get_target_obj(e)) == 0) ? field::Alliance::RED
                                                                          : field::Alliance::BLUE;
  g_robot_selected = false;
  restart_preview();
}

void route_cb(lv_event_t* e) {
  const uint32_t i = lv_dropdown_get_selected(lv_event_get_target_obj(e));
  g_selected = static_cast<Route>(i < ROUTE_COUNT ? i : 0);
  g_robot_selected = false;
  restart_preview();
}

void start_cb(lv_event_t* e) {
  const uint32_t i = lv_dropdown_get_selected(lv_event_get_target_obj(e));
  g_start_sel = static_cast<int>(i < START_COUNT ? i : 0);
  restart_preview();
}

// The add dropdown stays showing "+ Add step"; picking an entry appends it.
void add_cb(lv_event_t* e) {
  lv_obj_t* dd = lv_event_get_target_obj(e);
  switch (lv_dropdown_get_selected(dd)) {
    case 0: push_step(Kind::DRIVE, 12.0f); break;
    case 1: push_step(Kind::TURN, 90.0f); break;
    case 2: push_step(Kind::INTAKE, 1.0f); break;
    case 3: push_step(Kind::INTAKE, -1.0f); break;
    case 4: push_step(Kind::CLAW, 1.0f); break;
    case 5: push_step(Kind::CLAW, 0.0f); break;
    case 6: push_step(Kind::LIFT, 0.8f); break;
    case 7: push_step(Kind::LIFT, 0.0f); break;
    case 8: push_step(Kind::WAIT, 500.0f); break;
    case 9: // "Score sequence": raise, spit out, drop back down
      push_step(Kind::LIFT, 0.8f);
      push_step(Kind::INTAKE, -1.0f);
      push_step(Kind::LIFT, 0.0f);
      break;
    default: break;
  }
  lv_dropdown_set_selected(dd, 0);
}

/// tag 0 = undo, 1 = clear
void edit_btn_cb(lv_event_t* e) {
  const int tag = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
  if (tag == 0 && g_custom.n > 0) --g_custom.n;
  else if (tag == 1) g_custom.n = 0;
  restart_preview();
}

// Tapping a step row cycles its value -- keeps the builder usable without
// steppers, which there is no screen space for.
void step_cb(lv_event_t* e) {
  const int i = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
  if (i >= g_custom.n) return;
  Step& s = g_custom.s[i];

  switch (s.kind) {
    case Kind::DRIVE: {
      static const float v[] = {6, 12, 18, 24, 36, -6, -12, -24};
      int k = 0;
      for (int j = 0; j < 8; ++j)
        if (std::fabs(v[j] - s.a) < 0.01f) k = j + 1;
      s.a = v[k % 8];
      break;
    }
    case Kind::TURN: {
      static const float v[] = {45, 90, 135, 180, -45, -90, -135};
      int k = 0;
      for (int j = 0; j < 7; ++j)
        if (std::fabs(v[j] - s.a) < 0.01f) k = j + 1;
      s.a = v[k % 7];
      break;
    }
    case Kind::GOTO: s.flag ^= F_SWERVE; break; // toggle turn-then-drive vs arc
    case Kind::INTAKE: s.a = (s.a > 0) ? -1.0f : ((s.a < 0) ? 0.0f : 1.0f); break;
    case Kind::CLAW: s.a = (s.a > 0.5f) ? 0.0f : 1.0f; break;
    case Kind::LIFT: s.a = (s.a >= 0.79f) ? 0.0f : s.a + 0.4f; break;
    case Kind::WAIT: s.a = (s.a >= 1900.0f) ? 250.0f : s.a * 2.0f; break;
  }
  restart_preview();
}

void rec_btn_cb(lv_event_t*) { g_req_rec_toggle = true; }

void trail_btn_cb(lv_event_t*) {
  g_trail_n = 0;
  redraw();
}

void canvas_cb(lv_event_t*) {
  lv_indev_t* indev = lv_indev_active();
  if (indev == nullptr) return;

  lv_point_t p;
  lv_indev_get_point(indev, &p);

  lv_area_t a;
  lv_obj_get_coords(g_canvas, &a);
  const float ix = field::in_x(static_cast<float>(p.x - a.x1));
  const float iy = field::in_y(static_cast<float>(p.y - a.y1));

  // tap a toggle -> flip which alliance owns that quadrant
  for (const field::Toggle& t : field::toggles) {
    const float dx = ix - t.x, dy = iy - t.y;
    if (dx * dx + dy * dy <= 64.0f) {
      field::cycle_toggle(t.quadrant);
      redraw();
      return;
    }
  }

  // tap the robot -> select it and show its coordinates
  const float rx = (g_view == View::LIVE) ? g_live_x : g_sim.x;
  const float ry = (g_view == View::LIVE) ? g_live_y : g_sim.y;
  const float dx = ix - rx, dy = iy - ry;
  if (dx * dx + dy * dy <= (ROBOT_IN * 0.75f) * (ROBOT_IN * 0.75f)) {
    g_robot_selected = !g_robot_selected;
    redraw();
    return;
  }

  g_robot_selected = false;

  // tap elsewhere in the editor -> drop a waypoint. Points are stored in the
  // RED frame so the blue mirror stays correct.
  if (g_view == View::EDIT) {
    g_selected = Route::CUSTOM;
    lv_dropdown_set_selected(g_dd_route, static_cast<uint32_t>(Route::CUSTOM));
    push_step(Kind::GOTO, mirrored() ? -ix : ix, iy);
    return;
  }
  redraw();
}

void anim_cb(lv_timer_t*) {
  // service cross-task requests on the UI task, where widget calls are safe
  if (g_req_live) {
    g_req_live = false;
    if (g_view != View::LIVE) set_view(View::LIVE);
  }
  if (g_req_rec_toggle) {
    g_req_rec_toggle = false;
    if (g_recording) {
      g_recording = false;
      if (g_rec_n >= 2) {
        recording_commit();
        lv_dropdown_set_selected(g_dd_route, static_cast<uint32_t>(Route::CUSTOM));
        refresh_steps();
      }
    } else {
      g_rec_n = 0;
      g_trail_n = 0;
      g_recording = true;
      if (g_view != View::LIVE) set_view(View::LIVE);
    }
  }

  live_sample(); // cheap, and keeps the trail warm whichever view is up

  if (g_view == View::SELECT || g_view == View::EDIT) sim_tick();
  redraw();
}

// ---------------------------------------------------------------------------
// Widget helpers
// ---------------------------------------------------------------------------

lv_obj_t* make_label(lv_obj_t* parent, int x, int y, const char* text, uint32_t color,
                     const lv_font_t* font) {
  lv_obj_t* t = lv_label_create(parent);
  lv_label_set_text(t, text);
  lv_obj_set_style_text_font(t, font, LV_PART_MAIN);
  lv_obj_set_style_text_color(t, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_pos(t, x, y);
  return t;
}

/// Small tracked-out caption above a control.
lv_obj_t* make_caption(lv_obj_t* parent, int x, int y, const char* text) {
  lv_obj_t* t = make_label(parent, x, y, text, ink::DIM, &lv_font_montserrat_10);
  lv_obj_set_style_text_letter_space(t, 2, LV_PART_MAIN);
  return t;
}

lv_obj_t* make_button(lv_obj_t* parent, int x, int y, int w, int h, const char* text, lv_event_cb_t cb,
                      int tag, uint32_t bg = ink::CTRL_HI, const lv_font_t* font = &lv_font_montserrat_14) {
  lv_obj_t* b = lv_button_create(parent);
  lv_obj_set_pos(b, x, y);
  lv_obj_set_size(b, w, h);
  lv_obj_set_style_radius(b, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(b, lv_color_hex(bg), LV_PART_MAIN);
  lv_obj_set_style_border_width(b, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(b, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_shadow_width(b, 0, LV_PART_MAIN);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, reinterpret_cast<void*>(static_cast<intptr_t>(tag)));

  lv_obj_t* t = lv_label_create(b);
  lv_label_set_text(t, text);
  lv_obj_set_style_text_font(t, font, LV_PART_MAIN);
  lv_obj_set_style_text_color(t, lv_color_hex(ink::TEXT), LV_PART_MAIN);
  lv_obj_center(t);
  return b;
}

lv_obj_t* make_dropdown(lv_obj_t* parent, int x, int y, int w, const char* options, lv_event_cb_t cb) {
  lv_obj_t* d = lv_dropdown_create(parent);
  lv_obj_set_pos(d, x, y);
  lv_obj_set_size(d, w, ROW_H);
  lv_dropdown_set_options_static(d, options);
  lv_obj_set_style_radius(d, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(d, lv_color_hex(ink::CTRL), LV_PART_MAIN);
  lv_obj_set_style_border_width(d, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(d, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_text_color(d, lv_color_hex(ink::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(d, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_pad_all(d, 7, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(d, 0, LV_PART_MAIN);

  lv_obj_t* list = lv_dropdown_get_list(d);
  lv_obj_set_style_bg_color(list, lv_color_hex(ink::CARD), LV_PART_MAIN);
  lv_obj_set_style_border_color(list, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_text_color(list, lv_color_hex(ink::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(list, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_radius(list, 6, LV_PART_MAIN);
  const lv_style_selector_t sel =
      static_cast<lv_style_selector_t>(LV_PART_SELECTED) | static_cast<lv_style_selector_t>(LV_STATE_CHECKED);
  lv_obj_set_style_bg_color(list, lv_color_hex(ink::ACCENT), sel);
  lv_obj_set_style_text_color(list, lv_color_hex(ink::BG), sel);

  lv_obj_add_event_cb(d, cb, LV_EVENT_VALUE_CHANGED, nullptr);
  return d;
}

/// Full-screen transparent container, one per view.
lv_obj_t* make_root(lv_obj_t* scr) {
  lv_obj_t* v = lv_obj_create(scr);
  lv_obj_set_pos(v, 0, 0);
  lv_obj_set_size(v, SCREEN_W, SCREEN_H);
  lv_obj_set_style_bg_opa(v, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(v, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(v, 0, LV_PART_MAIN);
  lv_obj_remove_flag(v, LV_OBJ_FLAG_SCROLLABLE);
  return v;
}

/// The left-hand card that the SELECT / EDIT / LIVE views live inside.
lv_obj_t* make_card(lv_obj_t* root) {
  lv_obj_t* c = lv_obj_create(root);
  lv_obj_set_pos(c, CARD_X, CARD_Y);
  lv_obj_set_size(c, CARD_W, CARD_H);
  lv_obj_set_style_bg_color(c, lv_color_hex(ink::CARD), LV_PART_MAIN);
  lv_obj_set_style_border_width(c, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(c, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_radius(c, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(c, CARD_PAD, LV_PART_MAIN);
  lv_obj_remove_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  return c;
}

/// Back chevron plus a section title, occupying the top 24 px of a card.
void make_header(lv_obj_t* card, const char* title) {
  make_button(card, 0, 0, 60, 24, LV_SYMBOL_LEFT "  Menu", nav_cb, static_cast<int>(View::LANDING), ink::CTRL,
              &lv_font_montserrat_12);
  lv_obj_t* t = make_label(card, 70, 4, title, ink::TEXT, &lv_font_montserrat_16);
  lv_obj_set_style_text_letter_space(t, 1, LV_PART_MAIN);
}

void make_rule(lv_obj_t* card, int y) {
  lv_obj_t* r = lv_obj_create(card);
  lv_obj_set_pos(r, 0, y);
  lv_obj_set_size(r, COL_W, 1);
  lv_obj_set_style_bg_color(r, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_border_width(r, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(r, 0, LV_PART_MAIN);
}

// ---------------------------------------------------------------------------
// View construction
// ---------------------------------------------------------------------------

void build_landing(lv_obj_t* scr) {
  lv_obj_t* root = make_root(scr);
  g_root[static_cast<int>(View::LANDING)] = root;

  // ---- logo ----
  // LOGO_BOX is the square the artwork has to fit inside. Anything wider runs
  // into the button column at RX.
  constexpr int LOGO_BOX = 148;
  constexpr int LX = 26, LY = 46;
#ifdef HAVE_LOGO_IMAGE
  lv_obj_t* img = lv_image_create(root);
  lv_image_set_src(img, &logo_img);
  // The badge is taller than it is wide, so centre it in the box rather than
  // pinning it to the left edge.
  const int iw = static_cast<int>(logo_img.header.w);
  const int ih = static_cast<int>(logo_img.header.h);
  lv_obj_set_pos(img, LX + (LOGO_BOX - iw) / 2, LY + (LOGO_BOX - ih) / 2);
#else
  // placeholder badge until a real logo is dropped in
  lv_obj_t* badge = lv_obj_create(root);
  lv_obj_set_pos(badge, LX, LY);
  lv_obj_set_size(badge, LOGO_BOX, LOGO_BOX);
  lv_obj_set_style_radius(badge, LOGO_BOX / 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(badge, lv_color_hex(ink::CARD), LV_PART_MAIN);
  lv_obj_set_style_border_width(badge, 2, LV_PART_MAIN);
  lv_obj_set_style_border_color(badge, lv_color_hex(ink::ACCENT), LV_PART_MAIN);
  lv_obj_remove_flag(badge, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t* bt = lv_label_create(badge);
  lv_label_set_text(bt, "LC");
  lv_obj_set_style_text_font(bt, &lv_font_montserrat_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(bt, lv_color_hex(ink::ACCENT), LV_PART_MAIN);
  lv_obj_center(bt);
#endif

  // ---- title block ----
  constexpr int RX = 196;
  lv_obj_t* h1 = make_label(root, RX, 34, "LUCKY CATS", ink::TEXT, &lv_font_montserrat_30);
  lv_obj_set_style_text_letter_space(h1, 1, LV_PART_MAIN);
  make_caption(root, RX + 2, 70, "V5RC OVERRIDE   2026-27");

  // ---- entry points ----
  constexpr int BW = 256;
  make_button(root, RX, 96, BW, 34, "Run a route", nav_cb, static_cast<int>(View::SELECT), ink::CTRL_HI,
              &lv_font_montserrat_16);
  make_button(root, RX, 136, BW, 34, "Design a route", nav_cb, static_cast<int>(View::EDIT), ink::CTRL_HI,
              &lv_font_montserrat_16);
  make_button(root, RX, 176, BW, 34, "Live telemetry", nav_cb, static_cast<int>(View::LIVE), ink::CTRL_HI,
              &lv_font_montserrat_16);

  make_label(root, LX + 20, LY + LOGO_BOX + 12, "LuckyCats  v0.0.1", ink::DIM, &lv_font_montserrat_10);
}

void build_select(lv_obj_t* scr) {
  lv_obj_t* root = make_root(scr);
  g_root[static_cast<int>(View::SELECT)] = root;
  lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* card = make_card(root);
  make_header(card, "RUN");

  make_caption(card, 2, 32, "ALLIANCE");
  g_dd_alliance = make_dropdown(card, 0, 44, COL_W, "Red\nBlue", alliance_cb);

  make_caption(card, 2, 80, "ROUTE");
  g_dd_route = make_dropdown(card, 0, 92, COL_W, ROUTE_OPTIONS, route_cb);

  make_caption(card, 2, 128, "START");
  g_dd_start = make_dropdown(card, 0, 140, COL_W, START_OPTIONS, start_cb);

  make_rule(card, 178);
  g_lbl_pose = make_label(card, 2, 184, "", ink::TEXT, &lv_font_montserrat_12);
  g_lbl_step = make_label(card, 2, 198, "", ink::ACCENT, &lv_font_montserrat_12);
}

void build_edit(lv_obj_t* scr) {
  lv_obj_t* root = make_root(scr);
  g_root[static_cast<int>(View::EDIT)] = root;
  lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* card = make_card(root);
  make_header(card, "DESIGN");

  g_dd_add = make_dropdown(card, 0, 32, COL_W,
                           "Drive\nTurn\nIntake in\nIntake out\nClaw grip\nClaw release\n"
                           "Lift up\nLift down\nWait\nScore sequence",
                           add_cb);
  lv_dropdown_set_text(g_dd_add, "+   Add step");

  lv_obj_t* list = lv_obj_create(card);
  lv_obj_set_pos(list, 0, 70);
  lv_obj_set_size(list, COL_W, 110);
  lv_obj_set_style_bg_color(list, lv_color_hex(ink::SUNK), LV_PART_MAIN);
  lv_obj_set_style_border_width(list, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(list, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_radius(list, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_all(list, 4, LV_PART_MAIN);

  g_lbl_empty = make_label(list, 4, 40, "Tap the field to\ndrop a waypoint.", ink::DIM,
                           &lv_font_montserrat_12);

  for (int i = 0; i < MAX_STEPS; ++i) {
    lv_obj_t* row = lv_button_create(list);
    lv_obj_set_size(row, COL_W - 20, 18);
    lv_obj_set_pos(row, 0, i * 21);
    lv_obj_set_style_radius(row, 4, LV_PART_MAIN);
    lv_obj_set_style_bg_color(row, lv_color_hex(ink::CTRL), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(row, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(row, step_cb, LV_EVENT_CLICKED, reinterpret_cast<void*>(static_cast<intptr_t>(i)));
    lv_obj_add_flag(row, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* lbl = lv_label_create(row);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl, lv_color_hex(ink::TEXT), LV_PART_MAIN);
    lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 2, 0);
    lv_label_set_text(lbl, "");

    g_step_row[i] = row;
    g_step_lbl[i] = lbl;
  }

  make_caption(card, 2, 186, "TAP A STEP TO EDIT IT");
  const int bw = (COL_W - 8) / 2;
  make_button(card, 0, 200, bw, BTN_H - 4, "Undo", edit_btn_cb, 0, ink::CTRL, &lv_font_montserrat_12);
  make_button(card, bw + 8, 200, bw, BTN_H - 4, "Clear", edit_btn_cb, 1, ink::CTRL, &lv_font_montserrat_12);
}

void build_live(lv_obj_t* scr) {
  lv_obj_t* root = make_root(scr);
  g_root[static_cast<int>(View::LIVE)] = root;
  lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* card = make_card(root);
  make_header(card, "LIVE");

  make_caption(card, 2, 32, "ODOMETRY");
  make_label(card, 2, 46, "X", ink::DIM, &lv_font_montserrat_12);
  g_lbl_lx = make_label(card, 34, 44, "0.0", ink::TEXT, &lv_font_montserrat_16);
  make_label(card, 2, 68, "Y", ink::DIM, &lv_font_montserrat_12);
  g_lbl_ly = make_label(card, 34, 66, "0.0", ink::TEXT, &lv_font_montserrat_16);
  make_label(card, 2, 90, "H", ink::DIM, &lv_font_montserrat_12);
  g_lbl_lh = make_label(card, 34, 88, "0", ink::TEXT, &lv_font_montserrat_16);

  make_rule(card, 114);

  make_label(card, 2, 122, "Battery", ink::DIM, &lv_font_montserrat_12);
  g_lbl_batt = make_label(card, 120, 122, "--", ink::GOOD, &lv_font_montserrat_12);
  make_label(card, 2, 140, "Recorder", ink::DIM, &lv_font_montserrat_12);
  g_lbl_rec = make_label(card, 120, 140, "idle", ink::DIM, &lv_font_montserrat_12);

  lv_obj_t* rb = make_button(card, 0, 158, COL_W, BTN_H, "Record route", rec_btn_cb, 0, ink::CTRL_HI,
                             &lv_font_montserrat_14);
  g_btn_rec_lbl = lv_obj_get_child(rb, 0);

  make_button(card, 0, 190, COL_W, BTN_H - 4, "Clear trail", trail_btn_cb, 0, ink::CTRL,
              &lv_font_montserrat_12);
}

} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void init() {
  lv_obj_t* scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(ink::BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // ---- field preview, shared by every view except the landing page ----
  g_canvas = lv_canvas_create(scr);
  lv_canvas_set_buffer(g_canvas, g_canvas_buf, field::PX, field::PX, LV_COLOR_FORMAT_ARGB8888);
  lv_obj_set_pos(g_canvas, FIELD_X, FIELD_Y);
  lv_obj_add_flag(g_canvas, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(g_canvas, canvas_cb, LV_EVENT_CLICKED, nullptr);
  build_background();

  build_landing(scr);
  build_select(scr);
  build_edit(scr);
  build_live(scr);

  set_view(View::LANDING);
  refresh_steps();
  sim_reset();

  // Must be an lv_timer, not a pros::Task: LVGL is serviced by its own daemon
  // and touching widgets from another task races with it.
  lv_timer_create(anim_cb, FRAME_MS, nullptr);
}

Route selected() { return g_selected; }

const char* selected_name() { return route_name(g_selected); }

field::Alliance alliance() { return g_alliance; }

void show_live() { g_req_live = true; }

void toggle_record() { g_req_rec_toggle = true; }

bool recording() { return g_recording; }

void run_selected() {
  show_live(); // watch the real robot track the plan

  float sx, sy, sth;
  start_pose(sx, sy, sth);
  chassis.setPose(sx, sy, sth);

  const int n = step_count();
  for (int i = 0; i < n; ++i) {
    const Step s = step_at(i);
    switch (s.kind) {
      case Kind::DRIVE: {
        const lemlib::Pose p = chassis.getPose();
        const float t = static_cast<float>(p.theta) * PI_F / 180.0f;
        lemlib::MoveToPointParams mp;
        mp.forwards = (s.a >= 0);
        chassis.moveToPoint(p.x + std::sin(t) * s.a, p.y + std::cos(t) * s.a, 3000, mp, false);
        break;
      }
      case Kind::TURN:
        chassis.turnToHeading(s.a, 2000, lemlib::TurnToHeadingParams{}, false);
        break;
      case Kind::GOTO: {
        const lemlib::Pose p = chassis.getPose();
        const float bear = bearing_to(static_cast<float>(p.x), static_cast<float>(p.y), s.a, s.b);
        if (s.flag & F_SWERVE) {
          // one boomerang motion: the robot arcs in and settles on the bearing
          chassis.moveToPose(s.a, s.b, bear, 4000, lemlib::MoveToPoseParams{}, false);
        } else {
          // default: square up to the point first, then drive straight at it
          chassis.turnToHeading(bear, 1500, lemlib::TurnToHeadingParams{}, false);
          chassis.moveToPoint(s.a, s.b, 4000, lemlib::MoveToPointParams{}, false);
        }
        break;
      }
      case Kind::INTAKE:
        intake.move(static_cast<int>(s.a) * 127);
        claw_spin.move(static_cast<int>(s.a) * 127);
        break;
      case Kind::CLAW:
        claw_pivot.move_absolute(s.a > 0.5f ? 0 : 400, 100);
        break;
      case Kind::LIFT:
        lift.move_absolute(s.a * 900.0f, 100); // TODO: calibrate lift travel in ticks
        break;
      case Kind::WAIT:
        pros::delay(static_cast<uint32_t>(s.a));
        break;
    }
    chassis.waitUntilDone();
  }

  intake.move(0);
  claw_spin.move(0);
}

} // namespace auton
