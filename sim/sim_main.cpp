// ---------------------------------------------------------------------------
// Desktop simulator entry point.
//
// Opens a 480x240 window -- the brain's exact screen -- and runs the real UI
// code from src/auton_selector.cpp and src/field.cpp against it. The mouse acts
// as the touchscreen. Function keys stand in for competition control.
//
// This replaces PROS's task scheduler, not the robot code: initialize(),
// autonomous() and opcontrol() are the same functions in src/main.cpp that the
// brain runs, called on the same kinds of threads.
// ---------------------------------------------------------------------------

#include <lvgl.h>

#include <atomic>
#include <cstdio>
#include <thread>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// Defined in src/main.cpp -- the actual competition entry points.
void initialize();
void autonomous();
void opcontrol();
void disabled();

namespace sim {
void tick_motors(double dt);

/// The simulator's own window. Keyboard state is global in Win32, so every key
/// read has to be gated on this or the sim reacts to typing in other apps --
/// which, before this existed, quietly killed the process when Esc was pressed
/// somewhere else entirely.
HWND g_hwnd = nullptr;

bool focused() { return g_hwnd != nullptr && GetForegroundWindow() == g_hwnd; }
} // namespace sim

namespace {

constexpr int BRAIN_W = 480;
constexpr int BRAIN_H = 240;
constexpr int ZOOM = 200; // percent -- 2x, so the window is 960x480

std::atomic<bool> g_auton_running{false};
std::atomic<bool> g_opcontrol_running{false};

void start_autonomous() {
  if (g_auton_running.exchange(true)) {
    std::printf("[sim] autonomous already running\n");
    return;
  }
  std::thread([] {
    std::printf("[sim] --- autonomous() ---\n");
    autonomous();
    std::printf("[sim] --- autonomous() returned ---\n");
    g_auton_running = false;
  }).detach();
}

void start_opcontrol() {
  if (g_opcontrol_running.exchange(true)) {
    std::printf("[sim] opcontrol already running\n");
    return;
  }
  std::thread([] {
    std::printf("[sim] --- opcontrol() --- (W/S and Up/Down drive, Space = button A)\n");
    opcontrol();
    g_opcontrol_running = false;
  }).detach();
}

bool edge(int vk, bool& prev) {
  const bool now = sim::focused() && (GetAsyncKeyState(vk) & 0x8000) != 0;
  const bool hit = now && !prev;
  prev = now;
  return hit;
}

void banner() {
  std::printf(
      "\n"
      "  LuckyCats brain simulator\n"
      "  ------------------------------------------------------------\n"
      "  Mouse            touchscreen\n"
      "  F1               run autonomous()\n"
      "  F2               run opcontrol()\n"
      "  W / S            left stick   (during opcontrol)\n"
      "  Up / Down        right stick  (during opcontrol)\n"
      "  Space            controller button A -- toggles route recording\n"
      "  Esc              quit\n"
      "  ------------------------------------------------------------\n"
      "  Simulated motion is constant-rate. It shows WHERE a route goes,\n"
      "  never how it will tune. PID, odometry drift and IMU behaviour are\n"
      "  hardware-only and are not modelled.\n\n");
}

} // namespace

int main() {
  banner();

  lv_init();

  lv_display_t* display = lv_windows_create_display(L"LuckyCats brain (simulated 480x240)", BRAIN_W,
                                                    BRAIN_H, ZOOM, false, true);
  if (display == nullptr) {
    std::printf("[sim] failed to create display\n");
    return 1;
  }
  lv_windows_acquire_pointer_indev(display);
  sim::g_hwnd = lv_windows_get_display_window_handle(display);

  // Same order as the brain: calibrate, then build the UI.
  initialize();

  bool p_f1 = false, p_f2 = false, p_esc = false;
  auto last = std::chrono::steady_clock::now();

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;

    sim::tick_motors(dt);

    if (edge(VK_F1, p_f1)) start_autonomous();
    if (edge(VK_F2, p_f2)) start_opcontrol();
    if (edge(VK_ESCAPE, p_esc)) break;

    const uint32_t wait = lv_timer_handler();
    Sleep(wait == LV_NO_TIMER_READY ? 5 : (wait > 20 ? 20 : wait));
  }

  std::printf("[sim] exit\n");
  // Robot threads are detached and may still be mid-motion; leave without
  // joining rather than hanging on a route that has seconds left to run.
  return 0;
}
