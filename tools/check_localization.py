#!/usr/bin/env python3
"""
Kiểm tra parity giữa bản tiếng Việt và bản tiếng Anh.

Nguyên tắc:
- Source thực thi duy nhất nằm trong /src.
- Bản source tiếng Anh trong /extras/source-en chỉ là reference.
- Ví dụ mặc định trong /examples dùng comment tiếng Việt.
- Bản /extras/examples-en dùng comment tiếng Anh.
- Sau khi bỏ comment và whitespace, code tương ứng phải giống nhau.
"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]


def strip_comments(src: str) -> str:
    """Bỏ // và /* */ nhưng vẫn giữ nguyên string/char literal."""
    out = []
    i = 0
    n = len(src)
    state = "code"

    while i < n:
        c = src[i]
        d = src[i + 1] if i + 1 < n else ""

        if state == "code":
            if c == '"':
                out.append(c)
                state = "str"
                i += 1
                continue
            if c == "'":
                out.append(c)
                state = "chr"
                i += 1
                continue
            if c == "/" and d == "/":
                state = "line"
                i += 2
                continue
            if c == "/" and d == "*":
                state = "block"
                i += 2
                continue

            out.append(c)
            i += 1
            continue

        if state == "str":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(src[i + 1])
                i += 2
                continue
            if c == '"':
                state = "code"
            i += 1
            continue

        if state == "chr":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(src[i + 1])
                i += 2
                continue
            if c == "'":
                state = "code"
            i += 1
            continue

        if state == "line":
            if c == "\n":
                out.append("\n")
                state = "code"
            i += 1
            continue

        if state == "block":
            if c == "*" and d == "/":
                state = "code"
                i += 2
            else:
                i += 1

    return "".join(out)


def canonical_code(path: Path) -> str:
    text = path.read_text(encoding="utf-8-sig")
    text = strip_comments(text)
    return re.sub(r"\s+", "", text)


def compare_pair(vi_path: Path, en_path: Path) -> bool:
    if not vi_path.exists():
        print(f"ERROR: thiếu file VI: {vi_path.relative_to(ROOT)}")
        return False
    if not en_path.exists():
        print(f"ERROR: thiếu file EN: {en_path.relative_to(ROOT)}")
        return False

    if canonical_code(vi_path) != canonical_code(en_path):
        print(
            "ERROR: code khác nhau sau khi bỏ comment: "
            f"{vi_path.relative_to(ROOT)} <-> {en_path.relative_to(ROOT)}"
        )
        return False

    print(
        "PASS: "
        f"{vi_path.relative_to(ROOT)} <-> {en_path.relative_to(ROOT)}"
    )
    return True


def main() -> int:
    ok = True

    source_pairs = [
        (
            ROOT / "src" / "TungLam_OmniMecanum_4WD.h",
            ROOT / "extras" / "source-en" / "TungLam_OmniMecanum_4WD.h",
        ),
        (
            ROOT / "src" / "TungLam_OmniMecanum_4WD.cpp",
            ROOT / "extras" / "source-en" / "TungLam_OmniMecanum_4WD.cpp",
        ),
        (
            ROOT / "src" / "TungLam_Control_MotorV5.h",
            ROOT / "extras" / "source-en" / "TungLam_Control_MotorV5.h",
        ),
    ]

    print("=== Source VI/EN parity ===")
    for vi_path, en_path in source_pairs:
        ok = compare_pair(vi_path, en_path) and ok

    print("\n=== Example VI/EN parity ===")
    vi_examples = sorted((ROOT / "examples").glob("*/*.ino"))
    if not vi_examples:
        print("ERROR: không tìm thấy Arduino examples")
        ok = False

    for vi_path in vi_examples:
        en_path = (
            ROOT
            / "extras"
            / "examples-en"
            / vi_path.parent.name
            / vi_path.name
        )
        ok = compare_pair(vi_path, en_path) and ok

    if ok:
        print("\nLocalization parity: PASS")
        return 0

    print("\nLocalization parity: FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())
