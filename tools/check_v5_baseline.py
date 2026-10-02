#!/usr/bin/env python3
"""Regression guard for the hardware-proven V5 baseline.

This script intentionally uses ASCII-only source literals so it is stable on
Windows and Linux regardless of console/code-page configuration.
"""

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8-sig")


def require(condition: bool, message: str) -> bool:
    if condition:
        print(f"PASS: {message}")
        return True
    print(f"ERROR: {message}")
    return False


def function_body(src: str, name: str) -> str:
    match = re.search(
        rf"constexpr\s+int32_t\s+{re.escape(name)}\s*\([^)]*\)\s*\{{(.*?)\}}",
        src,
        flags=re.S,
    )
    return match.group(1) if match else ""


def main() -> int:
    ok = True

    cpp = read("src/TungLam_OmniMecanum_4WD.cpp")
    header_en = read("extras/source-en/TungLam_OmniMecanum_4WD.h")
    readme_vi = read("README.md")
    readme_en = read("README.en.md")
    baseline_vi = read("extras/V5_BASELINE.md")
    baseline_en = read("extras/V5_BASELINE.en.md")
    ps2 = read("examples/PS2RobotControl/PS2RobotControl.ino")

    print("=== Modern Mecanum mixer parity with V5 ===")
    expected = {
        "mecanumM1": "return vx - vy - wz;",
        "mecanumM2": "return vx + vy - wz;",
        "mecanumM3": "return vx - vy + wz;",
        "mecanumM4": "return vx + vy + wz;",
    }
    for name, line in expected.items():
        body = " ".join(function_body(cpp, name).split())
        ok = require(line in body, f"{name}: {line}") and ok

    print("\n=== Physical wheel mapping ===")

    # Vietnamese baseline: match table structure and PWM values without
    # embedding accented text in this script.
    vi_rows = (
        (1, "D5"),
        (2, "D6"),
        (3, "D7"),
        (4, "D8"),
    )
    for wheel, pwm in vi_rows:
        pattern = rf"\|\s*M{wheel}\s*\|[^|]+\|\s*{pwm}\s*\|"
        ok = require(
            re.search(pattern, baseline_vi) is not None,
            f"VI baseline contains M{wheel} mapping with PWM {pwm}",
        ) and ok

    en_rows = (
        ("M1", "front-left", "D5"),
        ("M2", "rear-left", "D6"),
        ("M3", "rear-right", "D7"),
        ("M4", "front-right", "D8"),
    )
    for motor, position, pwm in en_rows:
        row = f"| {motor} | {position} | {pwm} |"
        ok = require(row in baseline_en, f"EN baseline: {row}") and ok

    ok = require(
        "M3 rear-right" in header_en and "M4 front-right" in header_en,
        "English API reference keeps M3 rear-right / M4 front-right",
    ) and ok

    # Vietnamese row is matched by shape only, avoiding accented literals.
    ok = require(
        re.search(r"\|\s*\+vy\s+[^|]+\|\s*-\s*\|\s*\+\s*\|\s*-\s*\|\s*\+\s*\|", readme_vi)
        is not None,
        "README VI keeps +vy left = - + - +",
    ) and ok

    ok = require(
        "| Strafe left (+vy) | - | + | - | + |" in readme_en,
        "README EN keeps +vy left = - + - +",
    ) and ok

    ok = require(
        "M1 -> M4 -> M3 -> M2" in baseline_vi
        and "M1 -> M4 -> M3 -> M2" in baseline_en,
        "clockwise physical order remains M1 -> M4 -> M3 -> M2",
    ) and ok

    print("\n=== PS2 recommended drive priority ===")
    right_idx = ps2.find("switch (ps2.rightDirection())")
    left_idx = ps2.find("switch (ps2.leftDirection())")

    ok = require(
        right_idx >= 0 and left_idx >= 0 and right_idx < left_idx,
        "right-stick arbitration is before left-stick translation",
    ) and ok

    right_section = ps2[right_idx:left_idx] if 0 <= right_idx < left_idx else ""

    ok = require(
        "robot.rotateLeft(TURN_SPEED);" in right_section
        and "robot.rotateRight(TURN_SPEED);" in right_section,
        "right stick commands rotation",
    ) and ok

    ok = require(
        right_section.count("return;") >= 2,
        "right-stick LEFT/RIGHT explicitly override translation",
    ) and ok

    print("\n=== Baseline documentation ===")
    ok = require(
        "M1 = vx - vy - wz" in baseline_vi
        and "M4 = vx + vy + wz" in baseline_vi,
        "VI baseline contains modern V5-parity mixer equations",
    ) and ok

    ok = require(
        "M1 = vx - vy - wz" in baseline_en
        and "M4 = vx + vy + wz" in baseline_en,
        "EN baseline contains modern V5-parity mixer equations",
    ) and ok

    if ok:
        print("\nV5 baseline regression: PASS")
        return 0

    print("\nV5 baseline regression: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
