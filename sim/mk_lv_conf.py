#!/usr/bin/env python3
"""Generate sim/lv_conf.h from the upstream LVGL 9.2.0 template.

Run:  python sim/mk_lv_conf.py

Keeping this as a script rather than a hand-maintained file means the simulator
config can be regenerated against a newer LVGL without re-deciding every knob,
and makes the deliberate differences from the brain explicit in one place.
"""

import os
import re
import sys

SIM = os.path.join(os.environ["LOCALAPPDATA"], "LuckyCatsSim").replace("\\", "/")
TEMPLATE = f"{SIM}/lvgl/lv_conf_template.h"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lv_conf.h")

# Matched to the brain wherever it matters. LV_CACHE_DEF_SIZE is the load-
# bearing one: the brain has no image cache, so lv_image widgets re-decode on
# every invalidate. Setting it higher here would hide that cost.
SET = {
    "LV_COLOR_DEPTH": "32",
    "LV_MEM_SIZE": "(10U * 1024U * 1024U)",
    "LV_CACHE_DEF_SIZE": "0",
    # Desktop-only: the brain has neither an OS abstraction nor a window.
    "LV_USE_OS": "LV_OS_WINDOWS",
    "LV_USE_WINDOWS": "1",
    # Louder than the brain on purpose -- a warning in the console beats a
    # blank screen with no explanation.
    "LV_USE_LOG": "1",
    "LV_LOG_PRINTF": "1",
    "LV_LOG_LEVEL": "LV_LOG_LEVEL_WARN",
    "LV_USE_ASSERT_NULL": "1",
    "LV_USE_ASSERT_MALLOC": "1",
}
for n in (10, 12, 14, 16, 18, 20, 24, 30, 36, 40, 42, 48):
    SET["LV_FONT_MONTSERRAT_%d" % n] = "1"

HEADER = """/**
 * LVGL config for the DESKTOP SIMULATOR only.
 *
 * The brain uses include/liblvgl/lv_conf.h and is not affected by this file.
 *
 * GENERATED -- do not edit by hand. Change sim/mk_lv_conf.py and re-run it.
 */
"""


def main():
    if not os.path.exists(TEMPLATE):
        sys.exit(f"template missing: {TEMPLATE}")

    s = open(TEMPLATE, encoding="utf-8-sig").read()  # utf-8-sig strips the BOM

    # The template ships inert behind an #if 0.
    s = s.replace('#if 0 /*Set it to "1" to enable content*/',
                  '#if 1 /*Set it to "1" to enable content*/', 1)

    missing = []
    for name, val in SET.items():
        pat = re.compile(r"^(\s*)#define\s+%s(\s+)\S.*$" % re.escape(name), re.M)
        if pat.search(s):
            s = pat.sub(lambda m: "%s#define %s %s" % (m.group(1), name, val), s, count=1)
        else:
            missing.append((name, val))

    if missing:
        add = "\n".join("#define %s %s" % kv for kv in missing)
        s = s.replace("#endif /*LV_CONF_H*/",
                      "/* --- not present in this LVGL version's template --- */\n%s\n\n#endif /*LV_CONF_H*/" % add)

    open(OUT, "w", newline="\n", encoding="utf-8").write(HEADER + s)
    print(f"wrote {OUT}")
    if missing:
        print("appended (absent from template):", [m[0] for m in missing])


if __name__ == "__main__":
    main()
