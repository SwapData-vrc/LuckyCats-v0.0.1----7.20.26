#!/usr/bin/env python3
"""Desktop simulator build.

Run from anywhere:   python sim/build.py          build
                     python sim/build.py run      build, then launch
                     python sim/build.py clean

A Makefile was tried first and does not work here: make reads the colon in
"C:/..." as a rule separator, and the project path contains spaces. This script
sidesteps both, and does mtime-based incremental compiles with a thread pool.

Nothing here touches the PROS build. The toolchain, the LVGL checkout and every
object file live under %LOCALAPPDATA%\\LuckyCatsSim, deliberately outside the
project -- the project is in OneDrive and would otherwise sync ~1 GB of build
output to the cloud.
"""

import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

SIM = os.path.join(os.environ["LOCALAPPDATA"], "LuckyCatsSim").replace("\\", "/")
CXX = f"{SIM}/mingw64/bin/g++.exe"
CC = f"{SIM}/mingw64/bin/gcc.exe"
LVGL = f"{SIM}/lvgl"
BUILD = f"{SIM}/build"
BIN = f"{BUILD}/luckycats_sim.exe"

HERE = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
PROJ = os.path.dirname(HERE)

# sim/shim MUST precede the project include dir -- it shadows main.h,
# lemlib/api.hpp and liblvgl/lvgl.h with the desktop stand-ins.
INCLUDES = [f"-I{HERE}", f"-I{HERE}/shim", f"-I{LVGL}", f"-I{PROJ}/include"]

WARN = ["-Wall", "-Wextra", "-Wno-unused-parameter", "-Wno-missing-field-initializers"]
# LUCKYCATS_SIM lets robot code tell the two builds apart. Only persistence uses
# it so far: the brain saves to /usd on the SD card, which does not exist here.
COMMON = ["-O1", "-g", "-DLV_CONF_INCLUDE_SIMPLE", "-DLUCKYCATS_SIM"] + WARN + INCLUDES
CXXFLAGS = COMMON + ["-std=gnu++20"]
CFLAGS = COMMON + ["-std=gnu11"]

LDFLAGS = ["-static", "-static-libgcc", "-static-libstdc++"]
LDLIBS = ["-luser32", "-lgdi32", "-lwinmm", "-lshcore", "-lole32", "-limm32"]


def lvgl_sources():
    out = []
    for root, _dirs, files in os.walk(f"{LVGL}/src"):
        for f in files:
            if f.endswith(".c"):
                out.append(os.path.join(root, f).replace("\\", "/"))
    return sorted(out)


def sources():
    # Robot code, compiled verbatim. subsystems.cpp is excluded on purpose:
    # sim_hw.cpp replaces it, because real PROS devices need a brain.
    proj = [
        f"{PROJ}/src/auton_selector.cpp",
        f"{PROJ}/src/field.cpp",
        f"{PROJ}/src/main.cpp",
        f"{PROJ}/src/field_img.c",
        f"{PROJ}/src/logo_img.c",
    ]
    sim = [f"{HERE}/sim_hw.cpp", f"{HERE}/sim_main.cpp"]
    return lvgl_sources() + proj + sim


def obj_for(src):
    # Flatten the drive letter and colons out of the object path.
    rel = src.replace(":", "").replace("//", "/")
    return f"{BUILD}/obj/{rel}.o"


def needs_build(src, obj):
    if not os.path.exists(obj):
        return True
    return os.path.getmtime(src) > os.path.getmtime(obj)


def compile_one(src):
    obj = obj_for(src)
    if not needs_build(src, obj):
        return (src, 0, "")
    os.makedirs(os.path.dirname(obj), exist_ok=True)
    tool, flags = (CC, CFLAGS) if src.endswith(".c") else (CXX, CXXFLAGS)
    cmd = [tool] + flags + ["-c", src, "-o", obj]
    p = subprocess.run(cmd, capture_output=True, text=True)
    return (src, p.returncode, (p.stderr or "") + (p.stdout or ""))


def build():
    if not os.path.exists(CXX):
        sys.exit(f"toolchain missing: {CXX}")
    if not os.path.isdir(LVGL):
        sys.exit(f"LVGL checkout missing: {LVGL}")

    srcs = sources()
    todo = [s for s in srcs if needs_build(s, obj_for(s))]
    print(f"[build] {len(srcs)} sources, {len(todo)} to compile")

    t0 = time.time()
    failed = 0
    done = 0
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        for src, rc, log in pool.map(compile_one, srcs):
            done += 1
            if log.strip():
                # Only surface diagnostics from our own code; LVGL's warnings
                # are not actionable here and drown everything else out.
                if not src.startswith(LVGL) or rc != 0:
                    print(f"--- {os.path.relpath(src, PROJ) if src.startswith(PROJ) else src}")
                    print(log.rstrip())
            if rc != 0:
                failed += 1
    if failed:
        sys.exit(f"[build] FAILED: {failed} file(s)")
    print(f"[build] compiled in {time.time() - t0:.1f}s")

    objs = [obj_for(s) for s in srcs]
    rsp = f"{BUILD}/link.rsp"
    with open(rsp, "w") as fh:
        fh.write("\n".join(f'"{o}"' for o in objs))
    p = subprocess.run([CXX] + LDFLAGS + ["-o", BIN, f"@{rsp}"] + LDLIBS,
                       capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stderr)
        sys.exit("[build] LINK FAILED")
    print(f"[build] linked {BIN} ({os.path.getsize(BIN) / 1e6:.1f} MB)")


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "build"
    if cmd == "clean":
        import shutil
        shutil.rmtree(BUILD, ignore_errors=True)
        print("[build] cleaned")
        return
    build()
    if cmd == "run":
        subprocess.run([BIN])


if __name__ == "__main__":
    main()
