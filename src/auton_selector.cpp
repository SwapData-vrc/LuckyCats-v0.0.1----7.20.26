#include "auton_selector.hpp" // IWYU pragma: keep
#include "field.hpp"          // IWYU pragma: keep

#include <cmath>
#include <cstdarg>
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
constexpr uint32_t BG_DEEP = 0x05070a; // intro / blackout backdrop
constexpr uint32_t CARD = 0x151a21;
constexpr uint32_t EDGE = 0x232a33;
constexpr uint32_t CTRL = 0x1d232b;
constexpr uint32_t CTRL_HI = 0x27303a;
constexpr uint32_t SUNK = 0x0e1218;
constexpr uint32_t TEXT = 0xe6edf3;
constexpr uint32_t DIM = 0x7d8590;
constexpr uint32_t ACCENT = 0x4cc9f0;
constexpr uint32_t VIOLET = 0xa987f5;
constexpr uint32_t TEAL = 0x2dd4bf;
constexpr uint32_t GOOD = 0x3fb950;
constexpr uint32_t WARN = 0xd29922;
constexpr uint32_t RED = 0xda3633;
constexpr uint32_t BLUE = 0x388bfd;
} // namespace ink

uint32_t alliance_ink(field::Alliance a) { return a == field::Alliance::BLUE ? ink::BLUE : ink::RED; }

/// Linear blend between two packed RGB colours. Used for pulses and fades --
/// LVGL has no colour-mixing helper that works on plain uint32_t hex.
uint32_t mix(uint32_t a, uint32_t b, float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  const int ar = static_cast<int>((a >> 16) & 0xff), ag = static_cast<int>((a >> 8) & 0xff),
            ab = static_cast<int>(a & 0xff);
  const int br = static_cast<int>((b >> 16) & 0xff), bg = static_cast<int>((b >> 8) & 0xff),
            bb = static_cast<int>(b & 0xff);
  const auto ch = [t](int u, int v) {
    return static_cast<uint32_t>(static_cast<float>(u) + (static_cast<float>(v - u)) * t) & 0xffu;
  };
  return (ch(ar, br) << 16) | (ch(ag, bg) << 8) | ch(ab, bb);
}

/// Wrap a heading difference into (-180, 180]. Every angular interpolation in
/// this file goes through here: without it a turn from 170 to -170 -- twenty
/// degrees -- animates the long way round, 340 degrees.
float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

/// Frame counter, for anything that breathes. Wraps harmlessly.
uint32_t g_tick = 0;

/// 0..1 triangle-ish pulse, `period` frames long.
float pulse(float period) {
  const float u = static_cast<float>(g_tick % static_cast<uint32_t>(period)) / period;
  return 0.5f - 0.5f * std::cos(u * 2.0f * PI_F);
}

// ---------------------------------------------------------------------------
// Debug log
//
// A ring of fixed-width slots. Writers can be any task -- autonomous, opcontrol
// or the UI -- and there is no lock, because PROS's Mutex is not available to
// the desktop simulator and a lock here would be the only thing forcing it.
//
// Fixed-width slots are what makes that safe rather than merely convenient: a
// writer can never run past the end of its own slot, so the worst case if two
// tasks log at the same instant is one garbled line, not corruption. `head`
// is published only after the line is complete, so a reader never sees half a
// line -- it can only miss one.
// ---------------------------------------------------------------------------

constexpr int LOG_LINES = 64; // ring depth
constexpr int LOG_COLS = 56;  // including the timestamp and the terminator
constexpr int LOG_VIS = 13;   // lines that fit on the console view

char g_log[LOG_LINES][LOG_COLS] = {};
volatile uint32_t g_log_head = 0;  // total lines ever written, not an index
volatile uint32_t g_log_shown = 0; // head as of the last console repaint

// ---------------------------------------------------------------------------
// Route steps
//
// These belong to the on-brain route builder -- the Design view and the driving
// recorder -- and to nothing else. Routines in AUTONS are compiled C++ and know
// nothing about any of this.
//
// It is deliberately private to this file. An earlier version exposed it as a
// vocabulary for writing routines in, which meant learning a second way to say
// moveToPoint that could only express what the preview happened to support.
// ---------------------------------------------------------------------------

enum class Kind : uint8_t {
  DRIVE,  // a = inches along current heading, negative reverses
  TURN,   // a = absolute heading, degrees
  GOTO,   // a,b = field point in inches; flag bit 0 = swerve
  INTAKE, // a = +1 in, -1 out, 0 stop
  CLAW,   // a = 1 close/grip, 0 open/release
  LIFT,   // a = 0..1 target height
  WAIT,   // a = milliseconds
  SCORE,  // a = lift height 0..1; raise, eject off the back, return to travel
};

/// GOTO modifier: without it the robot turns to the bearing first and then
/// drives straight; with it the robot arcs to the point in one motion.
constexpr uint8_t F_SWERVE = 1;

struct Step {
  Kind kind;
  float a, b;
  uint8_t flag;
};

/// Where the lift sits while driving, and full cascade travel in ticks. Kept in
/// step with the copies in src/autons.cpp -- these are only used by the builder.
constexpr float LIFT_TRAVEL = 0.15f;
constexpr float LIFT_TICKS = 900.0f;

/// Cap on the hand-built route.
constexpr int MAX_STEPS = 20;

struct RouteBuf {
  Step s[MAX_STEPS];
  int n;
};

// ---------------------------------------------------------------------------
// Starting positions
//
// Absolute field positions, not red-frame: tapping the north wall puts the
// robot against the north wall whichever alliance is selected. Only the
// hand-built route is mirrored, because it is drawn on the field in the red
// frame; a compiled routine runs exactly as written.
// ---------------------------------------------------------------------------

struct Start {
  float x, y, th;
};

const Start STARTS[] = {
    {0.0f, 0.0f, 0.0f},      // 0  use the route's own default
    {-52.0f, 0.0f, 90.0f},   // 1  west quadrant, facing +X
    {0.0f, 52.0f, 180.0f},   // 2  north quadrant, facing -Y
    {52.0f, 0.0f, 270.0f},   // 3  east quadrant, facing -X
    {0.0f, -52.0f, 0.0f},    // 4  south quadrant, facing +Y
    {-52.0f, 44.0f, 90.0f},  // 5  loader +Y
    {-52.0f, -44.0f, 90.0f}, // 6  loader -Y
    {0.0f, 0.0f, 0.0f},      // 7  field centre
};
constexpr int START_COUNT = 8;

// Indices into STARTS. The four quadrant entries are what a tap on a Toggle
// selects, so they are named rather than spelled as magic numbers.
constexpr int ST_DEFAULT = 0;
constexpr int ST_WEST = 1;
constexpr int ST_NORTH = 2;
constexpr int ST_EAST = 3;
constexpr int ST_SOUTH = 4;

const char* const START_OPTIONS = "Route default\nWest quadrant\nNorth quadrant\nEast quadrant\n"
                                  "South quadrant\nLoader +Y\nLoader -Y\nField centre";

// ---------------------------------------------------------------------------
// Routes
//
// The routes themselves are in src/autons.cpp. This file only indexes them.
//
// A selection is an int in [0, AUTON_COUNT]: below AUTON_COUNT it names an
// entry in AUTONS, and AUTON_COUNT itself means the hand-built Custom route.
// That keeps the dropdown, the saved file and this code using one number, and
// it means AUTONS can be empty without a special case anywhere except the
// dropdown text.
// ---------------------------------------------------------------------------

/// The selection value that means the hand-built route rather than an entry in
/// AUTONS. It sits one past the end, so a plain int covers both cases.
int custom_sel() { return AUTON_COUNT; }

/// Dropdown text, built once at init from the names in AUTONS. lv_dropdown
/// keeps the pointer when set with _static, so this has to outlive the widget --
/// hence file scope rather than a local in build_select.
char g_route_options[MAX_AUTONS * 24 + 24] = {};

void build_route_options() {
  int p = 0;
  for (int i = 0; i < AUTON_COUNT && i < MAX_AUTONS; ++i) {
    const int left = static_cast<int>(sizeof(g_route_options)) - p;
    if (left <= 1) break;
    p += std::snprintf(g_route_options + p, static_cast<size_t>(left), "%s\n", AUTONS[i].name);
    if (p >= static_cast<int>(sizeof(g_route_options))) {
      p = static_cast<int>(sizeof(g_route_options)) - 1; // snprintf truncated
      break;
    }
  }
  std::snprintf(g_route_options + p, sizeof(g_route_options) - static_cast<size_t>(p), "Custom route");
}

// The custom route. RAM only -- never written to flash, so it is gone on power
// cycle. It is kept after a run so you can re-run the same test.
RouteBuf g_custom{{}, 0};
float g_custom_start[3] = {-52.0f, 0.0f, 90.0f};

/// Index into AUTONS, or custom_sel() for the hand-built route. Starts on the
/// first route if there is one; with an empty AUTONS that is Custom.
int g_selected = 0;
field::Alliance g_alliance = field::Alliance::RED;
int g_start_sel = 0;

/// Blackout hides the whole selection behind an idle-looking screen so a scout
/// standing over the pit cannot read the route off the brain. Persisted on
/// purpose: a brownout mid-event must not come back up revealing everything.
bool g_blackout = false;

bool is_custom() { return g_selected >= AUTON_COUNT; }

/// Steps only exist for the hand-built route. A compiled routine is a function;
/// there is nothing here to count, list, animate or estimate.
int step_count() { return is_custom() ? g_custom.n : 0; }

bool mirrored() { return g_alliance == field::Alliance::BLUE; }

/// A step as it will actually be driven, after alliance mirroring. Mirroring
/// applies to the hand-built route only -- it is drawn on the field in the red
/// frame, so it has to flip. A compiled routine runs exactly as written.
Step step_at(int i) {
  Step s = g_custom.s[i];
  if (mirrored()) {
    // mirror across the Y axis: x negates, and a clockwise heading becomes
    // the same magnitude counter-clockwise. The TURN case must be re-wrapped:
    // negating 180 gives -180, which is the same bearing but compares as a
    // full turn away from it.
    if (s.kind == Kind::TURN) s.a = wrap180(-s.a);
    else if (s.kind == Kind::GOTO) s.a = -s.a;
  }
  return s;
}

void start_pose(float& x, float& y, float& th) {
  if (g_start_sel != 0) {
    // Explicit override from the START dropdown. Absolute field coordinates --
    // not mirrored, because the point of tapping a quadrant is to put the robot
    // in the quadrant that was tapped.
    const Start& s = STARTS[g_start_sel];
    x = s.x;
    y = s.y;
    th = s.th;
    return;
  }

  if (is_custom()) {
    // The hand-built route is authored in the red frame, so its start mirrors
    // with the rest of it.
    x = mirrored() ? -g_custom_start[0] : g_custom_start[0];
    y = g_custom_start[1];
    th = mirrored() ? wrap180(-g_custom_start[2]) : g_custom_start[2];
    return;
  }

  // A compiled routine's start pose is used exactly as the table gives it. If a
  // routine wants to differ by side it can ask alliance() and setPose itself.
  const Auton& a = AUTONS[g_selected];
  x = a.start_x;
  y = a.start_y;
  th = a.start_heading;
}

// ---------------------------------------------------------------------------
// Quadrants as the start picker
//
// Tapping a Toggle puts the robot against that wall. STARTS holds absolute
// field positions, so this is a straight mapping in both directions -- it used
// to swap east and west for blue, back when start poses were mirrored.
// ---------------------------------------------------------------------------

/// STARTS index that puts the robot in the on-screen quadrant `q`.
int start_for_quad(field::Quad q) {
  switch (q) {
    case field::Quad::NORTH: return ST_NORTH;
    case field::Quad::SOUTH: return ST_SOUTH;
    case field::Quad::EAST: return ST_EAST;
    case field::Quad::WEST: return ST_WEST;
  }
  return ST_DEFAULT;
}

/// On-screen quadrant the current start selection lands in, or -1 if the
/// selection is not one of the four quadrant entries.
int highlight_quad() {
  switch (g_start_sel) {
    case ST_NORTH: return static_cast<int>(field::Quad::NORTH);
    case ST_SOUTH: return static_cast<int>(field::Quad::SOUTH);
    case ST_EAST: return static_cast<int>(field::Quad::EAST);
    case ST_WEST: return static_cast<int>(field::Quad::WEST);
    default: return -1;
  }
}

// ---------------------------------------------------------------------------
// Persistence
//
// The brain reboots between matches and after every brownout. Without this the
// selection is silently lost and whatever route happens to be first runs
// instead, which is a match-losing failure mode that looks like nothing went
// wrong. Everything the user chose is written to the SD card as plain text so
// it can also be inspected or fixed on a laptop.
//
// No SD card is not an error -- fopen simply fails and the selector runs with
// defaults, exactly as it did before this existed.
// ---------------------------------------------------------------------------

#ifdef LUCKYCATS_SIM
constexpr const char* SAVE_PATH = "luckycats_sim_auton.txt";
#else
constexpr const char* SAVE_PATH = "/usd/auton.txt";
#endif

constexpr uint32_t SAVE_DEBOUNCE_MS = 1500;

bool g_save_dirty = false;
uint32_t g_save_mark = 0;

const char* kind_tag(Kind k) {
  switch (k) {
    case Kind::DRIVE: return "DRIVE";
    case Kind::TURN: return "TURN";
    case Kind::GOTO: return "GOTO";
    case Kind::INTAKE: return "INTAKE";
    case Kind::CLAW: return "CLAW";
    case Kind::LIFT: return "LIFT";
    case Kind::WAIT: return "WAIT";
    case Kind::SCORE: return "SCORE";
  }
  return "WAIT";
}

bool kind_from_tag(const char* s, Kind& out) {
  struct Row {
    const char* tag;
    Kind k;
  };
  static const Row rows[] = {{"DRIVE", Kind::DRIVE}, {"TURN", Kind::TURN},     {"GOTO", Kind::GOTO},
                             {"INTAKE", Kind::INTAKE}, {"CLAW", Kind::CLAW}, {"LIFT", Kind::LIFT},
                             {"WAIT", Kind::WAIT},     {"SCORE", Kind::SCORE}};
  for (const Row& r : rows) {
    if (std::strcmp(r.tag, s) == 0) {
      out = r.k;
      return true;
    }
  }
  return false;
}

void save_now() {
  std::FILE* f = std::fopen(SAVE_PATH, "w");
  if (f == nullptr) return;

  std::fprintf(f, "# LuckyCats selector state, format 1. Safe to edit by hand;\n");
  std::fprintf(f, "# unrecognised lines are ignored. Delete the file to reset.\n");
  std::fprintf(f, "alliance %d\n", g_alliance == field::Alliance::BLUE ? 1 : 0);
  std::fprintf(f, "route %d\n", static_cast<int>(g_selected));
  std::fprintf(f, "start %d\n", g_start_sel);
  std::fprintf(f, "blackout %d\n", g_blackout ? 1 : 0);
  std::fprintf(f, "customstart %.2f %.2f %.2f\n", static_cast<double>(g_custom_start[0]),
               static_cast<double>(g_custom_start[1]), static_cast<double>(g_custom_start[2]));
  for (int i = 0; i < g_custom.n; ++i) {
    const Step& s = g_custom.s[i];
    std::fprintf(f, "step %s %.3f %.3f %u\n", kind_tag(s.kind), static_cast<double>(s.a),
                 static_cast<double>(s.b), static_cast<unsigned>(s.flag));
  }
  std::fclose(f);
}

void load_saved() {
  std::FILE* f = std::fopen(SAVE_PATH, "r");
  if (f == nullptr) return;

  char line[128];
  int n = 0;
  bool first = true;
  while (std::fgets(line, sizeof(line), f) != nullptr) {
    char* p = line;
    // Notepad and PowerShell both write a UTF-8 BOM. Without this the first
    // line silently fails to parse and only the first setting is lost, which is
    // a genuinely baffling thing to debug. The file is documented as hand
    // editable, so it has to survive being hand edited.
    if (first) {
      first = false;
      if (static_cast<unsigned char>(p[0]) == 0xEF && static_cast<unsigned char>(p[1]) == 0xBB &&
          static_cast<unsigned char>(p[2]) == 0xBF)
        p += 3;
    }
    int iv = 0;
    float a = 0, b = 0, c = 0;
    unsigned fl = 0;
    char tag[16];

    if (std::sscanf(p, "alliance %d", &iv) == 1) {
      g_alliance = iv ? field::Alliance::BLUE : field::Alliance::RED;
    } else if (std::sscanf(p, "route %d", &iv) == 1) {
      if (iv >= 0 && iv <= AUTON_COUNT) g_selected = iv;
    } else if (std::sscanf(p, "start %d", &iv) == 1) {
      if (iv >= 0 && iv < START_COUNT) g_start_sel = iv;
    } else if (std::sscanf(p, "blackout %d", &iv) == 1) {
      g_blackout = (iv != 0);
    } else if (std::sscanf(p, "customstart %f %f %f", &a, &b, &c) == 3) {
      g_custom_start[0] = a;
      g_custom_start[1] = b;
      g_custom_start[2] = c;
    } else if (std::sscanf(p, "step %15s %f %f %u", tag, &a, &b, &fl) == 4) {
      Kind k;
      if (kind_from_tag(tag, k) && n < MAX_STEPS)
        g_custom.s[n++] = Step{k, a, b, static_cast<uint8_t>(fl)};
    }
  }
  g_custom.n = n;
  std::fclose(f);

  // A saved CUSTOM selection with nothing in it would leave the user staring at
  // an empty preview with no obvious cause.
  if (is_custom() && g_custom.n == 0) g_selected = 0;
}

/// Note a change without writing yet. Every tap would otherwise hit the SD card
/// mid-frame, and a dropdown scrub is dozens of taps.
void mark_dirty() {
  g_save_dirty = true;
  g_save_mark = pros::millis();
}

void save_tick() {
  if (!g_save_dirty) return;
  if (pros::millis() - g_save_mark < SAVE_DEBOUNCE_MS) return;
  g_save_dirty = false;
  save_now();
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
    case Kind::SCORE: std::snprintf(out, n, "Score   at %.0f%%", static_cast<double>(s.a * 100.0f)); break;
  }
}

/// step_text into a shared buffer, for log lines. Not reentrant, and only ever
/// called from the one autonomous task.
const char* step_desc(const Step& s) {
  static char buf[40];
  step_text(s, buf, sizeof(buf));
  return buf;
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

// SCORE is two legs. These are guesses at cascade travel time and want
// measuring against the real lift -- they are what makes the route estimate
// under the ROUTE dropdown optimistic or pessimistic.
constexpr uint32_t SCORE_RAISE_MS = 600;
constexpr uint32_t SCORE_EJECT_MS = 700;

// How long the preview holds on a step that does not move the robot. Also what
// the route estimate charges for them, which is why they are constants rather
// than literals buried in begin_step.
constexpr uint32_t INTAKE_MS = 420;
constexpr uint32_t CLAW_MS = 360;
constexpr uint32_t LIFT_MS = 650;

struct Sim {
  int step_i;
  int phase; // GOTO only: 0 = turning to bearing, 1 = driving
  uint32_t t_in_step;
  uint32_t dur; // length of the current leg, ms
  uint32_t hold;
  float x, y, th;    // live pose
  float sx, sy, sth; // pose when the current leg began
  float tx, ty;      // position the current leg is heading for
  float dth;         // signed shortest turn for this leg, degrees
  float lift;        // 0..1
  float lift_from, lift_to;
  int intake;    // -1 out, 0 stop, +1 in
  bool claw;     // true = closed / gripping
  bool to_start; // pre-roll: sliding to a newly chosen start pose
  bool finished;
};

Sim g_sim{};
bool g_robot_selected = false;

/// Leg durations. Both scale with how far the robot actually has to go -- a
/// fixed duration made a 5 degree nudge and a 180 degree spin take the same
/// time, which is the main reason turns read wrong in the preview.
uint32_t drive_ms(float inches) { return static_cast<uint32_t>(std::fabs(inches) * 34.0f) + 240; }
uint32_t turn_ms(float degrees) { return static_cast<uint32_t>(std::fabs(degrees) * 4.2f) + 180; }

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

/// Aim the current leg. `tth` is an absolute heading; the turn is stored as the
/// shortest signed delta from where the robot is now, so interpolation never
/// takes the long way round.
void set_leg(float tx, float ty, float tth, uint32_t dur) {
  g_sim.t_in_step = 0;
  g_sim.sx = g_sim.x;
  g_sim.sy = g_sim.y;
  g_sim.sth = g_sim.th;
  g_sim.tx = tx;
  g_sim.ty = ty;
  g_sim.dth = wrap180(tth - g_sim.th);
  g_sim.lift_from = g_sim.lift;
  g_sim.lift_to = g_sim.lift;
  g_sim.dur = dur ? dur : 1;
}

void begin_step() {
  g_sim.phase = 0;
  set_leg(g_sim.x, g_sim.y, g_sim.th, 1);

  if (g_sim.step_i < 0 || g_sim.step_i >= step_count()) return;
  const Step s = step_at(g_sim.step_i);

  switch (s.kind) {
    case Kind::DRIVE: {
      const float t = g_sim.th * PI_F / 180.0f;
      set_leg(g_sim.x + std::sin(t) * s.a, g_sim.y + std::cos(t) * s.a, g_sim.th, drive_ms(s.a));
      break;
    }
    case Kind::TURN: {
      const float d = wrap180(s.a - g_sim.th);
      set_leg(g_sim.x, g_sim.y, g_sim.th + d, turn_ms(d));
      break;
    }
    case Kind::GOTO: {
      const float bear = bearing_to(g_sim.x, g_sim.y, s.a, s.b);
      const float d = wrap180(bear - g_sim.th);
      const float dist = std::sqrt((s.a - g_sim.x) * (s.a - g_sim.x) + (s.b - g_sim.y) * (s.b - g_sim.y));
      if (s.flag & F_SWERVE) {
        // one continuous arc: position and heading resolve together, so the
        // leg has to be long enough for whichever of the two dominates
        g_sim.phase = 1;
        set_leg(s.a, s.b, g_sim.th + d, drive_ms(dist) + turn_ms(d) / 2);
      } else {
        // phase 0 turns in place; phase 1 drives the straight line
        set_leg(g_sim.x, g_sim.y, g_sim.th + d, turn_ms(d));
      }
      break;
    }
    case Kind::INTAKE:
      g_sim.intake = static_cast<int>(s.a);
      set_leg(g_sim.x, g_sim.y, g_sim.th, INTAKE_MS);
      break;
    case Kind::CLAW:
      g_sim.claw = (s.a > 0.5f);
      set_leg(g_sim.x, g_sim.y, g_sim.th, CLAW_MS);
      break;
    case Kind::LIFT:
      set_leg(g_sim.x, g_sim.y, g_sim.th, LIFT_MS);
      g_sim.lift_to = s.a; // after set_leg -- it resets lift_to to the current height
      break;
    case Kind::WAIT:
      set_leg(g_sim.x, g_sim.y, g_sim.th, static_cast<uint32_t>(s.a));
      break;
    case Kind::SCORE:
      // phase 0 raises; sim_tick starts phase 1, which ejects and comes back
      // down. Two phases rather than one so the preview shows the lift going up
      // before anything leaves the robot, which is the order that matters when
      // checking a Goal is tall enough.
      set_leg(g_sim.x, g_sim.y, g_sim.th, SCORE_RAISE_MS);
      g_sim.lift_to = s.a; // after set_leg, which resets it
      break;
  }
}

// ---------------------------------------------------------------------------
// Route estimate
//
// Walks the route without animating it and adds up the same per-leg durations
// the preview uses, so the SELECT card can say whether a route fits in the
// period before anyone drives it. It inherits every limitation of the preview
// -- constant rate, no PID settling, no slew -- so read it as "this route is
// nowhere near 15 s" or "this is going to be tight", never as a real time.
// ---------------------------------------------------------------------------

uint32_t g_route_ms = 0;

/// Match autonomous is 15 s; programming skills is a minute.
/// Match autonomous. The hand-built route is the only thing estimated, and it
/// is always a match route -- skills gets written in C++ like everything else.
constexpr uint32_t BUDGET_MS = 15000;

/// Wall-clock length of the last real run, or 0. A measurement beats an
/// estimate, and it is the only number available for a compiled routine.
uint32_t g_last_run_ms = 0;

uint32_t route_ms() {
  float x, y, th;
  start_pose(x, y, th);

  uint32_t total = 0;
  const int n = step_count();
  for (int i = 0; i < n; ++i) {
    const Step s = step_at(i);
    switch (s.kind) {
      case Kind::DRIVE: {
        const float t = th * PI_F / 180.0f;
        x += std::sin(t) * s.a;
        y += std::cos(t) * s.a;
        total += drive_ms(s.a);
        break;
      }
      case Kind::TURN: {
        const float d = wrap180(s.a - th);
        th = wrap180(th + d);
        total += turn_ms(d);
        break;
      }
      case Kind::GOTO: {
        const float bear = bearing_to(x, y, s.a, s.b);
        const float d = wrap180(bear - th);
        const float dist = std::sqrt((s.a - x) * (s.a - x) + (s.b - y) * (s.b - y));
        total += (s.flag & F_SWERVE) ? drive_ms(dist) + turn_ms(d) / 2 : turn_ms(d) + drive_ms(dist);
        x = s.a;
        y = s.b;
        th = wrap180(bear);
        break;
      }
      // INTAKE, CLAW and LIFT cost nothing here on purpose. run_selected issues
      // them and moves straight on -- move() and move_absolute() do not block,
      // and the waitUntilDone() that follows is waiting on the chassis, which
      // is not moving. The preview holds on them (INTAKE_MS and friends) only
      // so a person watching can see them happen; charging the estimate for
      // that dwell would put every route seconds over its real cost.
      case Kind::INTAKE:
      case Kind::CLAW:
      case Kind::LIFT: break;
      case Kind::WAIT: total += static_cast<uint32_t>(s.a); break;
      case Kind::SCORE: total += SCORE_RAISE_MS + SCORE_EJECT_MS; break;
    }
  }
  return total;
}

/// Restart the preview. When `animate` is set the robot slides from wherever it
/// currently sits to the new start pose before the route begins, which is what
/// makes tapping a quadrant read as "the robot moved there" rather than a jump.
void sim_begin(bool animate) {
  g_route_ms = route_ms();

  for (int i = 0; i < 4; ++i) field::toggle_owner[i] = field::Alliance::NEUTRAL;

  const float ox = g_sim.x, oy = g_sim.y, oth = g_sim.th;
  float nx, ny, nth;
  start_pose(nx, ny, nth);

  g_sim = Sim{};

  if (!animate) {
    g_sim.x = nx;
    g_sim.y = ny;
    g_sim.th = nth;
    begin_step();
    return;
  }

  g_sim.x = ox;
  g_sim.y = oy;
  g_sim.th = oth;
  g_sim.to_start = true;
  g_sim.step_i = -1;

  const float dist = std::sqrt((nx - ox) * (nx - ox) + (ny - oy) * (ny - oy));
  const float d = wrap180(nth - oth);
  set_leg(nx, ny, oth + d, static_cast<uint32_t>(drive_ms(dist) * 0.75f) + turn_ms(d) / 2);
}

void sim_reset() { sim_begin(false); }

float ease(float u) { return u * u * (3.0f - 2.0f * u); }

/// Advance the current leg to normalised progress `u` (0..1, already eased).
void apply_leg(float u) {
  g_sim.x = g_sim.sx + (g_sim.tx - g_sim.sx) * u;
  g_sim.y = g_sim.sy + (g_sim.ty - g_sim.sy) * u;
  g_sim.th = g_sim.sth + g_sim.dth * u;
  g_sim.lift = g_sim.lift_from + (g_sim.lift_to - g_sim.lift_from) * u;
}

/// True once the current leg's clock has run out. Advances that clock.
bool leg_step(float& u_out) {
  g_sim.t_in_step += FRAME_MS;
  const float raw = static_cast<float>(g_sim.t_in_step) / static_cast<float>(g_sim.dur);
  u_out = ease(raw > 1.0f ? 1.0f : raw);
  return raw >= 1.0f;
}

void sim_tick() {
  if (g_sim.finished) {
    if (g_sim.hold > END_HOLD_MS) sim_reset();
    else g_sim.hold += FRAME_MS;
    return;
  }

  // pre-roll: sliding to a start pose the user just picked
  if (g_sim.to_start) {
    float u;
    const bool done = leg_step(u);
    apply_leg(u);
    if (!done) return;
    g_sim.th = wrap180(g_sim.th);
    g_sim.to_start = false;
    g_sim.step_i = 0;
    begin_step();
    return;
  }

  if (g_sim.step_i >= step_count()) {
    g_sim.finished = true;
    g_sim.hold = 0;
    return;
  }

  const Step s = step_at(g_sim.step_i);

  float u;
  const bool done = leg_step(u);
  apply_leg(u);

  if (g_sim.intake != 0) try_toggle();

  if (!done) return;

  apply_leg(1.0f);
  g_sim.th = wrap180(g_sim.th);

  // a non-swerve GOTO has finished turning; now drive the straight leg
  if (s.kind == Kind::GOTO && g_sim.phase == 0) {
    g_sim.phase = 1;
    const float dist = std::sqrt((s.a - g_sim.x) * (s.a - g_sim.x) + (s.b - g_sim.y) * (s.b - g_sim.y));
    set_leg(s.a, s.b, g_sim.th, drive_ms(dist));
    return;
  }

  // SCORE has finished raising; now eject off the back and come back down
  if (s.kind == Kind::SCORE && g_sim.phase == 0) {
    g_sim.phase = 1;
    g_sim.intake = -1;
    set_leg(g_sim.x, g_sim.y, g_sim.th, SCORE_EJECT_MS);
    g_sim.lift_to = LIFT_TRAVEL;
    return;
  }
  if (s.kind == Kind::SCORE) g_sim.intake = 0;

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

  g_selected = custom_sel();
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
  // No null check on FIELD_IMAGE_SRC: it is the address of a linked-in object,
  // so it can never be null and the compiler says so. The decoder call below is
  // the real guard -- it fails if the art is missing or malformed.
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

enum class View : uint8_t { LANDING = 0, SELECT = 1, EDIT = 2, LIVE = 3, CONSOLE = 4 };
constexpr int VIEW_COUNT = 5;

View g_view = View::LANDING;
lv_obj_t* g_root[VIEW_COUNT] = {};

// SELECT widgets
lv_obj_t* g_dd_alliance = nullptr;
lv_obj_t* g_dd_route = nullptr;
lv_obj_t* g_dd_start = nullptr;
lv_obj_t* g_lbl_pose = nullptr;
lv_obj_t* g_lbl_step = nullptr;
lv_obj_t* g_lbl_est = nullptr;

// EDIT widgets
lv_obj_t* g_dd_add = nullptr;
lv_obj_t* g_step_row[MAX_STEPS] = {nullptr};
lv_obj_t* g_step_lbl[MAX_STEPS] = {nullptr};
lv_obj_t* g_lbl_empty = nullptr;

// Pose HUD, floating over the field on every view that shows the field
lv_obj_t* g_hud = nullptr;
lv_obj_t* g_hud_stripe = nullptr;
lv_obj_t* g_hud_x = nullptr;
lv_obj_t* g_hud_y = nullptr;
lv_obj_t* g_hud_h = nullptr;

/// Set while autonomous() is actually driving the robot, so the HUD can say so.
volatile bool g_auton_active = false;

// LIVE widgets
lv_obj_t* g_lbl_lx = nullptr;
lv_obj_t* g_lbl_ly = nullptr;
lv_obj_t* g_lbl_lh = nullptr;
lv_obj_t* g_lbl_batt = nullptr;
lv_obj_t* g_lbl_rec = nullptr;
lv_obj_t* g_btn_rec_lbl = nullptr;

// CONSOLE widgets
lv_obj_t* g_lbl_log = nullptr;
lv_obj_t* g_lbl_log_count = nullptr;

// LANDING widgets
lv_obj_t* g_lbl_health = nullptr;

// Overlays. Neither is a View: both sit on top of whatever the selector is
// already showing, and blackout in particular has to survive a view switch
// requested by autonomous() starting.
lv_obj_t* g_intro = nullptr;
lv_obj_t* g_intro_bar_a = nullptr;
lv_obj_t* g_intro_bar_b = nullptr;
lv_obj_t* g_intro_logo = nullptr;
lv_obj_t* g_intro_title = nullptr;
lv_obj_t* g_intro_sub = nullptr;
lv_obj_t* g_intro_rule = nullptr;
bool g_intro_active = false;
uint32_t g_intro_t = 0;

lv_obj_t* g_black = nullptr;
lv_obj_t* g_black_hint = nullptr;

void intro_start();
void intro_tick();
void intro_finish();
void apply_blackout();
void blackout_enter_cb(lv_event_t* e);
void build_intro(lv_obj_t* scr);
void build_blackout(lv_obj_t* scr);
void build_hud(lv_obj_t* scr);
void update_hud();

const char* route_name(int r) {
  if (r < 0 || r >= AUTON_COUNT) return "Custom route";
  return AUTONS[r].name;
}

/// The console and the landing page are the two views that own the full width,
/// so the field preview has to get out of the way for both.
bool canvas_visible() {
  return g_view != View::LANDING && g_view != View::CONSOLE && !g_intro_active && !g_blackout;
}

void set_view(View v) {
  g_view = v;
  // An overlay owns the screen outright: the view still changes underneath so
  // that dismissing the overlay lands somewhere sensible, but nothing shows.
  const bool covered = g_intro_active || g_blackout;
  for (int i = 0; i < VIEW_COUNT; ++i) {
    if (g_root[i] == nullptr) continue;
    if (!covered && i == static_cast<int>(v)) {
      lv_obj_remove_flag(g_root[i], LV_OBJ_FLAG_HIDDEN);
      // Brief fade so switching views reads as a transition, not a cut.
      // lv_obj_fade_in drives LV_STYLE_OPA, which children inherit -- do not
      // pre-set opa_layered here, that is a different property and the fade
      // would never touch it, leaving the view permanently invisible.
      lv_obj_fade_in(g_root[i], 160, 0);
    } else {
      lv_obj_add_flag(g_root[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
  if (g_canvas != nullptr) {
    if (canvas_visible()) lv_obj_remove_flag(g_canvas, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(g_canvas, LV_OBJ_FLAG_HIDDEN);
  }
  if (g_hud != nullptr) {
    if (canvas_visible()) lv_obj_remove_flag(g_hud, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(g_hud, LV_OBJ_FLAG_HIDDEN);
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
    if (!is_custom()) {
      // There is nothing to animate: a compiled routine is a function, and the
      // only way to see its path is to run it. Say so rather than showing an
      // idle field that looks like the preview is broken.
      std::snprintf(buf, sizeof(buf), "compiled routine - run to trace");
    } else if (g_sim.finished || n == 0) {
      std::snprintf(buf, sizeof(buf), "%s  -  %d step%s", route_name(g_selected), n, n == 1 ? "" : "s");
    } else {
      char b[40];
      step_text(step_at(g_sim.step_i), b, sizeof(b));
      std::snprintf(buf, sizeof(buf), "%d/%d  %s", g_sim.step_i + 1, n, b);
    }
    lv_label_set_text(g_lbl_step, buf);
  }

  if (g_lbl_est != nullptr) {
    uint32_t shown;
    bool measured;
    if (is_custom()) {
      shown = g_route_ms; // estimated from the steps
      measured = false;
    } else {
      shown = g_last_run_ms; // the only honest number for a compiled routine
      measured = true;
    }

    if (shown == 0) {
      lv_label_set_text(g_lbl_est, measured ? "not run yet" : "");
      lv_obj_set_style_text_color(g_lbl_est, lv_color_hex(ink::DIM), LV_PART_MAIN);
    } else {
      // Amber past the period. For an estimate that is optimistic amber: it has
      // no PID settling in it. For a measurement it is simply the truth.
      const bool over = shown > BUDGET_MS;
      std::snprintf(buf, sizeof(buf), "%s%s%.1f s", over ? "OVER  " : "", measured ? "" : "~",
                    static_cast<double>(shown) / 1000.0);
      lv_label_set_text(g_lbl_est, buf);
      lv_obj_set_style_text_color(g_lbl_est, lv_color_hex(over ? ink::WARN : ink::DIM), LV_PART_MAIN);
    }
  }
}

/// Rebuild the console text, but only when there is something new to show --
/// this runs at 20 fps and lv_label_set_text reflows the whole block.
void update_console() {
  if (g_lbl_log == nullptr) return;
  const uint32_t head = g_log_head;
  if (head == g_log_shown) return;
  g_log_shown = head;

  if (head == 0) {
    lv_label_set_text(g_lbl_log, "(nothing logged yet)");
  } else {
    const uint32_t first = (head > LOG_VIS) ? head - LOG_VIS : 0;
    char buf[LOG_VIS * LOG_COLS + 8];
    int p = 0;
    for (uint32_t i = first; i < head && p < static_cast<int>(sizeof(buf)) - 2; ++i) {
      if (i > first) buf[p++] = '\n';
      const char* line = g_log[i % LOG_LINES];
      p += std::snprintf(buf + p, sizeof(buf) - static_cast<size_t>(p), "%s", line);
    }
    buf[sizeof(buf) - 1] = '\0';
    lv_label_set_text(g_lbl_log, buf);
  }

  if (g_lbl_log_count != nullptr) {
    char c[32];
    std::snprintf(c, sizeof(c), "%lu line%s", static_cast<unsigned long>(head), head == 1 ? "" : "s");
    lv_label_set_text(g_lbl_log_count, c);
  }
}

// ---------------------------------------------------------------------------
// Pose HUD
//
// X / Y / heading overlaid on the field itself. The card readouts are only
// visible on the view that owns them, and during a real run the screen is on
// LIVE -- but the field is what you are actually looking at while the robot
// moves, so the numbers belong there too.
// ---------------------------------------------------------------------------

void update_hud() {
  if (g_hud == nullptr) return;

  const bool live = (g_view == View::LIVE);
  const float x = live ? g_live_x : g_sim.x;
  const float y = live ? g_live_y : g_sim.y;
  const float th = live ? g_live_th : g_sim.th;

  char buf[24];
  std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(x));
  lv_label_set_text(g_hud_x, buf);
  std::snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(y));
  lv_label_set_text(g_hud_y, buf);
  std::snprintf(buf, sizeof(buf), "%.0f", static_cast<double>(wrap180(th)));
  lv_label_set_text(g_hud_h, buf);

  // The stripe says where the numbers come from: amber while autonomous is
  // actually driving, teal for real odometry, accent for a preview.
  uint32_t c = ink::ACCENT;
  if (g_auton_active) c = mix(ink::WARN, ink::TEXT, pulse(20.0f) * 0.5f);
  else if (live) c = ink::TEAL;
  else if (!g_sim.finished) c = mix(ink::ACCENT, ink::VIOLET, pulse(46.0f));
  lv_obj_set_style_bg_color(g_hud_stripe, lv_color_hex(c), LV_PART_MAIN);

  const uint32_t vc = g_auton_active ? ink::TEXT : ink::TEXT;
  lv_obj_set_style_text_color(g_hud_x, lv_color_hex(vc), LV_PART_MAIN);
  lv_obj_set_style_text_color(g_hud_y, lv_color_hex(vc), LV_PART_MAIN);
  lv_obj_set_style_text_color(g_hud_h, lv_color_hex(c), LV_PART_MAIN);
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
  field::draw_toggles(&layer, highlight_quad(), pulse(34.0f));

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

  update_hud();
  if (g_view == View::LIVE) update_live_readout();
  else update_readout();
}

/// `animate` slides the robot to the new start pose instead of teleporting it.
void restart_preview(bool animate = false) {
  refresh_steps();
  sim_begin(animate);
  redraw();
  mark_dirty();
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
  logf("alliance %s", mirrored() ? "BLUE" : "RED");
}

void route_cb(lv_event_t* e) {
  const uint32_t i = lv_dropdown_get_selected(lv_event_get_target_obj(e));
  g_selected = (static_cast<int>(i) <= AUTON_COUNT) ? static_cast<int>(i) : custom_sel();
  g_robot_selected = false;
  restart_preview();
  if (is_custom()) {
    logf("route Custom  %d steps  ~%.1f s", step_count(), static_cast<double>(g_route_ms) / 1000.0);
    if (g_route_ms > BUDGET_MS)
      logf("  WARNING: estimate over %lu s", static_cast<unsigned long>(BUDGET_MS / 1000));
  } else {
    logf("route %s", route_name(g_selected));
  }
}

void start_cb(lv_event_t* e) {
  const uint32_t i = lv_dropdown_get_selected(lv_event_get_target_obj(e));
  g_start_sel = static_cast<int>(i < START_COUNT ? i : 0);
  restart_preview(true); // slide there, so the change is visible on the field
}

/// Tapping a quadrant picks it as the start. Called from the canvas handler.
void select_quad(field::Quad q) {
  const int idx = start_for_quad(q);
  if (g_start_sel == idx) return;
  g_start_sel = idx;
  if (g_dd_start != nullptr) lv_dropdown_set_selected(g_dd_start, static_cast<uint32_t>(idx));
  g_robot_selected = false;
  restart_preview(true);
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
    case 9: push_step(Kind::SCORE, 0.45f); break;
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
    case Kind::SCORE: s.a = (s.a >= 0.79f) ? 0.30f : s.a + 0.15f; break;
  }
  restart_preview();
}

void rec_btn_cb(lv_event_t*) { g_req_rec_toggle = true; }

void log_clear_cb(lv_event_t*) {
  log_clear();
  update_console();
}

void trail_btn_cb(lv_event_t*) {
  g_trail_n = 0;
  redraw();
}

/// Field coordinates of a touch, or false if there is no active input device.
bool canvas_point(float& ix, float& iy) {
  lv_indev_t* indev = lv_indev_active();
  if (indev == nullptr) return false;

  lv_point_t p;
  lv_indev_get_point(indev, &p);

  lv_area_t a;
  lv_obj_get_coords(g_canvas, &a);
  ix = field::in_x(static_cast<float>(p.x - a.x1));
  iy = field::in_y(static_cast<float>(p.y - a.y1));
  return true;
}

/// Which Toggle a touch is closest to, or nullptr.
const field::Toggle* toggle_at(float ix, float iy) {
  constexpr float R2 = field::TOGGLE_HIT_IN * field::TOGGLE_HIT_IN;
  const field::Toggle* best = nullptr;
  float best_d2 = R2;
  for (const field::Toggle& t : field::toggles) {
    const float dx = ix - t.x, dy = iy - t.y;
    const float d2 = dx * dx + dy * dy;
    if (d2 <= best_d2) {
      best_d2 = d2;
      best = &t;
    }
  }
  return best;
}

// Holding a Toggle cycles who owns that quadrant. That used to be the plain tap,
// but the tap is now the start picker -- ownership is a preview detail and the
// start pose is what actually gets driven, so the start pose won the short
// gesture.
void canvas_long_cb(lv_event_t*) {
  float ix, iy;
  if (!canvas_point(ix, iy)) return;
  const field::Toggle* t = toggle_at(ix, iy);
  if (t == nullptr) return;
  field::cycle_toggle(t->quadrant);
  redraw();
}

void canvas_cb(lv_event_t*) {
  float ix, iy;
  if (!canvas_point(ix, iy)) return;

  // tap a toggle -> start the route from that quadrant
  if (const field::Toggle* t = toggle_at(ix, iy)) {
    select_quad(t->quadrant);
    return;
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
    g_selected = custom_sel();
    lv_dropdown_set_selected(g_dd_route, static_cast<uint32_t>(custom_sel()));
    push_step(Kind::GOTO, mirrored() ? -ix : ix, iy);
    return;
  }
  redraw();
}

void anim_cb(lv_timer_t*) {
  ++g_tick;

  // service cross-task requests on the UI task, where widget calls are safe
  if (g_req_live) {
    g_req_live = false;
    // Blackout outranks this. Autonomous starting must not be what reveals the
    // route to the team standing behind the field.
    if (g_blackout) {
      g_view = View::LIVE;
    } else {
      intro_finish(); // a match has started; the title sequence is over
      if (g_view != View::LIVE) set_view(View::LIVE);
    }
  }
  if (g_req_rec_toggle) {
    g_req_rec_toggle = false;
    if (g_recording) {
      g_recording = false;
      if (g_rec_n >= 2) {
        recording_commit();
        lv_dropdown_set_selected(g_dd_route, static_cast<uint32_t>(custom_sel()));
        refresh_steps();
        logf("recorded %d waypoints into Custom", g_custom.n);
      } else {
        logf("recording discarded: %d waypoints", g_rec_n);
      }
    } else {
      g_rec_n = 0;
      g_trail_n = 0;
      g_recording = true;
      if (g_view != View::LIVE) set_view(View::LIVE);
      logf("recording started");
    }
  }

  live_sample(); // cheap, and keeps the trail warm whichever view is up
  save_tick();

  if (g_intro_active) {
    intro_tick();
    return;
  }

  if (g_blackout) {
    // Slow breath on the standby line, so the screen looks alive rather than
    // frozen -- a frozen brain is the thing someone walks over to investigate.
    if (g_black_hint != nullptr)
      lv_obj_set_style_text_color(g_black_hint,
                                  lv_color_hex(mix(0x3a424c, ink::DIM, pulse(78.0f))), LV_PART_MAIN);
    return;
  }

  if (g_view == View::CONSOLE) {
    update_console();
    return; // no field, no preview -- nothing else on this view moves
  }

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
  // The list is clamped to the screen, not to the card, so its height is fixed
  // by where the dropdown sits -- about 96 px for the ROUTE row. Nine routes in
  // 96 px is three visible rows at the default 14 pt, which is a lot of
  // dragging on a resistive screen. Dropping the list to 12 pt with tighter
  // rows fits five without making the options hard to read.
  lv_obj_set_style_max_height(list, 200, LV_PART_MAIN);
  lv_obj_set_style_pad_ver(list, 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(list, lv_color_hex(ink::CARD), LV_PART_MAIN);
  lv_obj_set_style_border_color(list, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_text_color(list, lv_color_hex(ink::TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(list, &lv_font_montserrat_12, LV_PART_MAIN);
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
  // A view root covers the whole screen and is built after the canvas, so it
  // sits on top of it. lv_obj_create is clickable by default, which means the
  // root silently ate every tap on the field -- waypoint drops and the quadrant
  // picker both looked dead. Children stay clickable; only the sheet does not.
  lv_obj_remove_flag(v, LV_OBJ_FLAG_CLICKABLE);
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
  // Four buttons at 32 px with a 4 px gap, which is the most that fits under
  // the title block without crowding the bottom edge.
  constexpr int BW = 256;
  constexpr int BH = 32;
  constexpr int B0 = 92;
  make_button(root, RX, B0, BW, BH, "Run a route", nav_cb, static_cast<int>(View::SELECT), ink::CTRL_HI,
              &lv_font_montserrat_16);
  make_button(root, RX, B0 + 36, BW, BH, "Design a route", nav_cb, static_cast<int>(View::EDIT), ink::CTRL_HI,
              &lv_font_montserrat_16);
  make_button(root, RX, B0 + 72, BW, BH, "Live telemetry", nav_cb, static_cast<int>(View::LIVE), ink::CTRL_HI,
              &lv_font_montserrat_16);
  make_button(root, RX, B0 + 108, BW, BH, "Console", nav_cb, static_cast<int>(View::CONSOLE), ink::CTRL,
              &lv_font_montserrat_16);

  make_label(root, LX + 20, LY + LOGO_BOX + 6, "LuckyCats  v0.0.1", ink::DIM, &lv_font_montserrat_10);
  // Filled in by the boot-time port check. Under the badge rather than on the
  // console, because a missing motor has to be visible without going looking.
  g_lbl_health = make_label(root, LX + 20, LY + LOGO_BOX + 20, "", ink::DIM, &lv_font_montserrat_10);

  // Blackout, tucked in the corner. Deliberately unlabelled and low contrast:
  // it is for the team, and an obvious "HIDE ROUTE" button is itself a tell.
  lv_obj_t* lock = make_button(root, 442, 6, 30, 22, LV_SYMBOL_EYE_CLOSE, blackout_enter_cb, 0, ink::CARD,
                               &lv_font_montserrat_12);
  lv_obj_set_style_border_color(lock, lv_color_hex(ink::CARD), LV_PART_MAIN);
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
  // Estimate lives on the caption row, not down with the step readout: it has
  // to be readable while the preview is mid-route, and the readout line is
  // busy naming the step that is currently running.
  g_lbl_est = make_label(card, 84, 80, "", ink::DIM, &lv_font_montserrat_10);
  lv_obj_set_width(g_lbl_est, COL_W - 84);
  lv_obj_set_style_text_align(g_lbl_est, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
  g_dd_route = make_dropdown(card, 0, 92, COL_W, g_route_options, route_cb);

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
                           "Lift up\nLift down\nWait\nScore",
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

/// Debug console. Full width rather than the usual left-hand card: log lines
/// are long, and wrapping them at 204 px would make the timestamps useless.
void build_console(lv_obj_t* scr) {
  lv_obj_t* root = make_root(scr);
  g_root[static_cast<int>(View::CONSOLE)] = root;
  lv_obj_add_flag(root, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* card = lv_obj_create(root);
  lv_obj_set_pos(card, CARD_X, CARD_Y);
  lv_obj_set_size(card, SCREEN_W - 2 * CARD_X, CARD_H);
  lv_obj_set_style_bg_color(card, lv_color_hex(ink::CARD), LV_PART_MAIN);
  lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(card, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_radius(card, 10, LV_PART_MAIN);
  lv_obj_set_style_pad_all(card, CARD_PAD, LV_PART_MAIN);
  lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);

  make_header(card, "CONSOLE");

  const int inner_w = SCREEN_W - 2 * CARD_X - 2 * CARD_PAD;
  g_lbl_log_count = make_label(card, inner_w - 138, 8, "0 lines", ink::DIM, &lv_font_montserrat_12);
  make_button(card, inner_w - 60, 0, 60, 24, "Clear", log_clear_cb, 0, ink::CTRL, &lv_font_montserrat_12);

  lv_obj_t* pane = lv_obj_create(card);
  lv_obj_set_pos(pane, 0, 32);
  lv_obj_set_size(pane, inner_w, CARD_H - 2 * CARD_PAD - 32);
  lv_obj_set_style_bg_color(pane, lv_color_hex(ink::SUNK), LV_PART_MAIN);
  lv_obj_set_style_border_width(pane, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(pane, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_radius(pane, 6, LV_PART_MAIN);
  lv_obj_set_style_pad_all(pane, 6, LV_PART_MAIN);
  lv_obj_remove_flag(pane, LV_OBJ_FLAG_SCROLLABLE);

  g_lbl_log = make_label(pane, 0, 0, "(nothing logged yet)", ink::TEXT, &lv_font_montserrat_12);
  lv_obj_set_style_text_line_space(g_lbl_log, 2, LV_PART_MAIN);
}

// ---------------------------------------------------------------------------
// Startup sequence
//
// Runs off the existing frame timer rather than blocking. That is not a style
// choice: init() is called from initialize(), and initialize() blocks every
// competition mode -- a sleep here would delay autonomous starting.
//
// Any touch skips to the end, and autonomous() starting kills it outright.
// ---------------------------------------------------------------------------

constexpr uint32_t I_SLIT = 420;   // hairline grows out of nothing
constexpr uint32_t I_OPEN = 980;   // it parts, logo pushes through
constexpr uint32_t I_TITLE = 1560; // wordmark
constexpr uint32_t I_SUB = 2040;   // subtitle and rule
constexpr uint32_t I_HOLD = 2500;  // beat
constexpr uint32_t I_END = 2820;   // faded out

/// Progress through [a, b) as 0..1, clamped outside it.
float seg(uint32_t t, uint32_t a, uint32_t b) {
  if (t <= a) return 0.0f;
  if (t >= b) return 1.0f;
  return static_cast<float>(t - a) / static_cast<float>(b - a);
}

void intro_finish() {
  if (!g_intro_active) return;
  g_intro_active = false;
  if (g_intro != nullptr) lv_obj_add_flag(g_intro, LV_OBJ_FLAG_HIDDEN);
  set_view(g_view);
  apply_blackout();
}

void intro_skip_cb(lv_event_t*) { intro_finish(); }

void intro_tick() {
  if (!g_intro_active) return;
  g_intro_t += FRAME_MS;
  const uint32_t t = g_intro_t;

  if (t >= I_END) {
    intro_finish();
    return;
  }

  // 1. a hairline opens out of the centre
  const float grow = ease(seg(t, 0, I_SLIT));
  const int half = static_cast<int>(6.0f + grow * 168.0f);

  // 2. it splits apart vertically, uncovering the badge
  const float part = ease(seg(t, I_SLIT, I_OPEN));
  const int spread = static_cast<int>(part * 62.0f);
  const lv_opa_t bar_opa = static_cast<lv_opa_t>(255.0f * (1.0f - part * 0.75f));

  for (int i = 0; i < 2; ++i) {
    lv_obj_t* bar = i ? g_intro_bar_b : g_intro_bar_a;
    const int dir = i ? 1 : -1;
    lv_obj_set_size(bar, half * 2, 2);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, dir * spread - 16);
    lv_obj_set_style_bg_opa(bar, bar_opa, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(mix(ink::ACCENT, ink::VIOLET, part)), LV_PART_MAIN);
  }

  // 3. the badge scales up to full size as it fades in
  const float rise = ease(seg(t, I_SLIT + 120, I_OPEN + 160));
  lv_image_set_scale(g_intro_logo, static_cast<uint32_t>(168.0f + rise * 88.0f));
  lv_obj_set_style_opa_layered(g_intro_logo, static_cast<lv_opa_t>(rise * 255.0f), LV_PART_MAIN);

  // 4. wordmark, tracking out as it arrives
  const float tt = ease(seg(t, I_OPEN, I_TITLE));
  lv_obj_set_style_text_opa(g_intro_title, static_cast<lv_opa_t>(tt * 255.0f), LV_PART_MAIN);
  lv_obj_set_style_text_letter_space(g_intro_title, static_cast<int32_t>(10.0f - tt * 8.0f), LV_PART_MAIN);

  // 5. rule sweeps out, subtitle follows
  const float st = ease(seg(t, I_TITLE, I_SUB));
  lv_obj_set_size(g_intro_rule, static_cast<int32_t>(st * 232.0f), 1);
  lv_obj_align(g_intro_rule, LV_ALIGN_CENTER, 0, 71);
  lv_obj_set_style_text_opa(g_intro_sub, static_cast<lv_opa_t>(st * 255.0f), LV_PART_MAIN);

  // 6. hand over
  const float out = seg(t, I_HOLD, I_END);
  lv_obj_set_style_opa_layered(g_intro, static_cast<lv_opa_t>(255.0f * (1.0f - out)), LV_PART_MAIN);
}

void intro_start() {
  if (g_intro == nullptr) return;
  g_intro_active = true;
  g_intro_t = 0;
  lv_obj_remove_flag(g_intro, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_opa_layered(g_intro, LV_OPA_COVER, LV_PART_MAIN);
  set_view(g_view); // hides the views underneath
  intro_tick();
}

void build_intro(lv_obj_t* scr) {
  g_intro = make_root(scr);
  lv_obj_set_style_bg_color(g_intro, lv_color_hex(ink::BG_DEEP), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_intro, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_flag(g_intro, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(g_intro, intro_skip_cb, LV_EVENT_CLICKED, nullptr);

  for (int i = 0; i < 2; ++i) {
    lv_obj_t* bar = lv_obj_create(g_intro);
    lv_obj_set_size(bar, 2, 2);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, -16);
    lv_obj_set_style_bg_color(bar, lv_color_hex(ink::ACCENT), LV_PART_MAIN);
    lv_obj_set_style_border_width(bar, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 1, LV_PART_MAIN);
    lv_obj_remove_flag(bar, LV_OBJ_FLAG_CLICKABLE);
    (i ? g_intro_bar_b : g_intro_bar_a) = bar;
  }

  g_intro_logo = lv_image_create(g_intro);
  lv_image_set_src(g_intro_logo, &logo_img);
  lv_image_set_inner_align(g_intro_logo, LV_IMAGE_ALIGN_CENTER);
  lv_obj_align(g_intro_logo, LV_ALIGN_CENTER, 0, -46);
  lv_obj_set_style_opa_layered(g_intro_logo, LV_OPA_TRANSP, LV_PART_MAIN);

  g_intro_title = make_label(g_intro, 0, 0, "LUCKY CATS", ink::TEXT, &lv_font_montserrat_30);
  lv_obj_align(g_intro_title, LV_ALIGN_CENTER, 0, 47);
  lv_obj_set_style_text_opa(g_intro_title, LV_OPA_TRANSP, LV_PART_MAIN);

  g_intro_rule = lv_obj_create(g_intro);
  lv_obj_set_size(g_intro_rule, 0, 1);
  lv_obj_align(g_intro_rule, LV_ALIGN_CENTER, 0, 71);
  lv_obj_set_style_bg_color(g_intro_rule, lv_color_hex(ink::ACCENT), LV_PART_MAIN);
  lv_obj_set_style_border_width(g_intro_rule, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(g_intro_rule, 0, LV_PART_MAIN);

  g_intro_sub = make_label(g_intro, 0, 0, "V5RC  OVERRIDE   2026-27", ink::ACCENT, &lv_font_montserrat_12);
  lv_obj_set_style_text_letter_space(g_intro_sub, 3, LV_PART_MAIN);
  lv_obj_align(g_intro_sub, LV_ALIGN_CENTER, 0, 85);
  lv_obj_set_style_text_opa(g_intro_sub, LV_OPA_TRANSP, LV_PART_MAIN);

  lv_obj_add_flag(g_intro, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------------
// Blackout
//
// Shows a plausible just-booted screen instead of the selection. It must look
// idle rather than deliberately hidden -- a black rectangle advertises that
// there is something worth hiding.
//
// Held down on the badge to leave, so a curious hand cannot get out of it by
// prodding the screen.
// ---------------------------------------------------------------------------

void apply_blackout() {
  if (g_black == nullptr) return;
  if (g_blackout && !g_intro_active) lv_obj_remove_flag(g_black, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_add_flag(g_black, LV_OBJ_FLAG_HIDDEN);
  set_view(g_view);
}

void blackout_enter_cb(lv_event_t*) {
  g_blackout = true;
  g_robot_selected = false;
  apply_blackout();
  save_now(); // immediately, not debounced: this one must survive a yank
}

void blackout_exit_cb(lv_event_t*) {
  g_blackout = false;
  apply_blackout();
  save_now();
}

void build_blackout(lv_obj_t* scr) {
  g_black = make_root(scr);
  lv_obj_set_style_bg_color(g_black, lv_color_hex(ink::BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_black, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_add_flag(g_black, LV_OBJ_FLAG_CLICKABLE); // swallow stray taps

  lv_obj_t* img = lv_image_create(g_black);
  lv_image_set_src(img, &logo_img);
  lv_obj_align(img, LV_ALIGN_LEFT_MID, 34, -6);
  lv_obj_add_flag(img, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(img, blackout_exit_cb, LV_EVENT_LONG_PRESSED, nullptr);

  lv_obj_t* h1 = make_label(g_black, 214, 82, "LUCKY CATS", ink::TEXT, &lv_font_montserrat_30);
  lv_obj_set_style_text_letter_space(h1, 1, LV_PART_MAIN);
  make_caption(g_black, 216, 118, "V5RC OVERRIDE   2026-27");

  g_black_hint = make_label(g_black, 216, 146, "standby", ink::DIM, &lv_font_montserrat_14);

  // Unhide, in the same corner as the eye icon that turned blackout on, so the
  // two read as one switch. Low contrast on purpose: it has to be findable by
  // someone who knows it is there without advertising that anything is hidden.
  // Holding the badge still works as well, for when the corner is smudged.
  lv_obj_t* unhide = make_button(g_black, 442, 6, 30, 22, LV_SYMBOL_EYE_OPEN, blackout_exit_cb, 0, ink::CARD,
                                 &lv_font_montserrat_12);
  lv_obj_set_style_border_color(unhide, lv_color_hex(ink::CARD), LV_PART_MAIN);

  lv_obj_add_flag(g_black, LV_OBJ_FLAG_HIDDEN);
}

/// The pose readout that floats over the bottom of the field. Parented to the
/// screen rather than to a view root, because it belongs to the canvas and the
/// canvas outlives every view.
void build_hud(lv_obj_t* scr) {
  // Corner block, not a full-width bar. The Toggles sit at the centre of each
  // wall and are now the quadrant picker, so anything spanning the width of the
  // field buries the south one.
  constexpr int W = 100;
  constexpr int H = 54;
  g_hud = lv_obj_create(scr);
  lv_obj_set_pos(g_hud, FIELD_X + 4, FIELD_Y + field::PX - H - 4);
  lv_obj_set_size(g_hud, W, H);
  lv_obj_set_style_bg_color(g_hud, lv_color_hex(ink::BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(g_hud, 225, LV_PART_MAIN);
  lv_obj_set_style_border_width(g_hud, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(g_hud, lv_color_hex(ink::EDGE), LV_PART_MAIN);
  lv_obj_set_style_radius(g_hud, 8, LV_PART_MAIN);
  lv_obj_set_style_pad_all(g_hud, 0, LV_PART_MAIN);
  lv_obj_remove_flag(g_hud, LV_OBJ_FLAG_SCROLLABLE);
  // must not eat touches -- the field underneath is the primary control
  lv_obj_remove_flag(g_hud, LV_OBJ_FLAG_CLICKABLE);

  g_hud_stripe = lv_obj_create(g_hud);
  lv_obj_set_pos(g_hud_stripe, 5, 6);
  lv_obj_set_size(g_hud_stripe, 3, H - 12);
  lv_obj_set_style_bg_color(g_hud_stripe, lv_color_hex(ink::ACCENT), LV_PART_MAIN);
  lv_obj_set_style_border_width(g_hud_stripe, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(g_hud_stripe, 2, LV_PART_MAIN);

  constexpr int LBL = 14, VAL = 36;
  make_caption(g_hud, LBL, 6, "X");
  make_caption(g_hud, LBL, 22, "Y");
  make_caption(g_hud, LBL, 38, "TH");
  g_hud_x = make_label(g_hud, VAL, 3, "0.0", ink::TEXT, &lv_font_montserrat_14);
  g_hud_y = make_label(g_hud, VAL, 19, "0.0", ink::TEXT, &lv_font_montserrat_14);
  g_hud_h = make_label(g_hud, VAL, 35, "0", ink::ACCENT, &lv_font_montserrat_14);
}

// ---------------------------------------------------------------------------
// Boot-time port check
//
// Walks the manifest in subsystems.cpp and reports anything the brain cannot
// see. This is the failure that costs matches: a motor knocked out of its port
// between rounds looks exactly like a tuning problem from the driver station,
// and it is invisible until the robot drives crooked.
//
// Probes with a throwaway device object rather than the real globals, because
// the check has to name the port that is empty, and the globals are groups.
// ---------------------------------------------------------------------------

void device_check() {
#ifdef LUCKYCATS_SIM
  // No smart ports on a PC. Saying so beats a green "all present" that means
  // nothing.
  logf("port check skipped: simulator");
  if (g_lbl_health != nullptr) {
    lv_label_set_text(g_lbl_health, "simulated hardware");
    lv_obj_set_style_text_color(g_lbl_health, lv_color_hex(ink::DIM), LV_PART_MAIN);
  }
#else
  int missing = 0;
  for (int i = 0; i < DEVICE_PORT_COUNT; ++i) {
    const DevicePort& d = DEVICE_PORTS[i];
    // The manifest carries the constructor's sign, which encodes reversal, not
    // a port number. Ports themselves are 1-21.
    const int p = (d.port < 0) ? -d.port : d.port;
    bool ok = false;
    switch (d.kind) {
      case DevKind::MOTOR: ok = pros::Motor(static_cast<std::int8_t>(p)).is_installed(); break;
      case DevKind::IMU: ok = pros::Imu(static_cast<std::uint8_t>(p)).is_installed(); break;
      case DevKind::ROTATION: ok = pros::Rotation(static_cast<std::int8_t>(p)).is_installed(); break;
    }
    if (!ok) {
      ++missing;
      logf("port %2d MISSING: %s", p, d.name);
    }
  }

  char msg[40];
  if (missing == 0) std::snprintf(msg, sizeof(msg), "%d ports OK", DEVICE_PORT_COUNT);
  else std::snprintf(msg, sizeof(msg), "%d of %d ports MISSING", missing, DEVICE_PORT_COUNT);
  logf("%s", msg);

  if (g_lbl_health != nullptr) {
    lv_label_set_text(g_lbl_health, msg);
    lv_obj_set_style_text_color(g_lbl_health, lv_color_hex(missing ? ink::RED : ink::GOOD), LV_PART_MAIN);
  }
#endif
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

  // Both before any widget exists: the route dropdown is built from the names
  // in AUTONS, and it should come up already showing what was chosen last time
  // rather than flickering to it afterwards.
  build_route_options();
  load_saved();
  // An empty AUTONS, or a saved index pointing past a route that has since been
  // deleted, both land on Custom rather than reading off the end of the table.
  if (g_selected < 0 || g_selected > AUTON_COUNT) g_selected = custom_sel();

  // ---- field preview, shared by every view except the landing page ----
  g_canvas = lv_canvas_create(scr);
  lv_canvas_set_buffer(g_canvas, g_canvas_buf, field::PX, field::PX, LV_COLOR_FORMAT_ARGB8888);
  lv_obj_set_pos(g_canvas, FIELD_X, FIELD_Y);
  lv_obj_add_flag(g_canvas, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(g_canvas, canvas_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(g_canvas, canvas_long_cb, LV_EVENT_LONG_PRESSED, nullptr);
  build_background();

  build_landing(scr);
  build_select(scr);
  build_edit(scr);
  build_live(scr);
  build_console(scr);
  build_hud(scr);
  build_blackout(scr);
  build_intro(scr);

  // Reflect the restored selection in the controls.
  lv_dropdown_set_selected(g_dd_alliance, g_alliance == field::Alliance::BLUE ? 1u : 0u);
  lv_dropdown_set_selected(g_dd_route, static_cast<uint32_t>(g_selected));
  lv_dropdown_set_selected(g_dd_start, static_cast<uint32_t>(g_start_sel));

  set_view(View::LANDING);
  refresh_steps();
  sim_reset();
  intro_start();
  apply_blackout();

  logf("boot: %s / %s", route_name(g_selected), mirrored() ? "BLUE" : "RED");
  if (g_blackout) logf("blackout restored from save");
  device_check();
  logf("battery %.0f%%", pros::battery::get_capacity());
  update_console();

  // Must be an lv_timer, not a pros::Task: LVGL is serviced by its own daemon
  // and touching widgets from another task races with it.
  lv_timer_create(anim_cb, FRAME_MS, nullptr);
}

int selected() { return g_selected; }

bool custom_selected() { return g_selected >= AUTON_COUNT; }

const char* selected_name() { return route_name(g_selected); }

field::Alliance alliance() { return g_alliance; }

void show_live() { g_req_live = true; }

void toggle_record() { g_req_rec_toggle = true; }

bool recording() { return g_recording; }

void logf(const char* fmt, ...) {
  const uint32_t head = g_log_head; // read once; ++ on a volatile is deprecated
  char* dst = g_log[head % LOG_LINES];

  // Timestamps are relative to program start, which is what matters when the
  // question is "how long did that leg take", and they are what makes the
  // console readable at a glance.
  const uint32_t ms = pros::millis();
  int p = std::snprintf(dst, LOG_COLS, "%3lu.%03lu  ", static_cast<unsigned long>(ms / 1000),
                        static_cast<unsigned long>(ms % 1000));
  if (p < 0) p = 0;
  if (p < LOG_COLS - 1) {
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(dst + p, static_cast<size_t>(LOG_COLS - p), fmt, ap);
    va_end(ap);
  }
  dst[LOG_COLS - 1] = '\0';

  // Also out the wire, so `pros terminal` gets the full history rather than the
  // last 64 lines the ring happens to be holding.
  std::printf("%s\n", dst);

  // Published last: a reader that samples head between these two statements
  // sees the previous line, never a half-written one.
  g_log_head = head + 1;
}

void log_clear() {
  g_log_head = 0;
  g_log_shown = 1; // force update_console to notice and repaint the empty state
}

void run_selected() {
  g_auton_active = true;
  show_live();  // the trail on this view is the record of what actually happened
  g_trail_n = 0; // start the trace clean, so it is this run and not the last one

  float sx, sy, sth;
  start_pose(sx, sy, sth);
  chassis.setPose(sx, sy, sth);

  const uint32_t t0 = pros::millis();
  logf("auton: %s / %s", route_name(g_selected), mirrored() ? "BLUE" : "RED");
  logf("start X %.1f Y %.1f H %.0f", static_cast<double>(sx), static_cast<double>(sy),
       static_cast<double>(sth));

  // A compiled routine is just a function call. Everything below it is the
  // interpreter for the hand-built route, which is the only thing that has
  // steps.
  if (!is_custom()) {
    const AutonFn fn = AUTONS[g_selected].run;
    if (fn != nullptr) fn();
    else logf("routine is null");

    chassis.waitUntilDone(); // in case the routine left a motion running
    intake.move(0);
    claw_spin.move(0);
    g_last_run_ms = pros::millis() - t0;
    g_auton_active = false;
    logf("auton done in %lu ms", static_cast<unsigned long>(g_last_run_ms));
    return;
  }

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
        lift.move_absolute(s.a * LIFT_TICKS, 100);
        break;
      case Kind::WAIT:
        pros::delay(static_cast<uint32_t>(s.a));
        break;
      case Kind::SCORE:
        // Front-to-back: the lift goes up, the rollers run backwards to push
        // the load out of the rear, then the lift returns to travel height. The
        // robot is already backed into the Goal by the DRIVE before this.
        lift.move_absolute(s.a * LIFT_TICKS, 100);
        pros::delay(SCORE_RAISE_MS);
        intake.move(-127);
        claw_spin.move(-127);
        pros::delay(SCORE_EJECT_MS);
        intake.move(0);
        claw_spin.move(0);
        lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
        break;
    }
    chassis.waitUntilDone();
    logf("%2d/%d  %s", i + 1, n, step_desc(s));
  }

  intake.move(0);
  claw_spin.move(0);
  g_last_run_ms = pros::millis() - t0;
  g_auton_active = false;
  logf("auton done in %lu ms", static_cast<unsigned long>(g_last_run_ms));
}

} // namespace auton
