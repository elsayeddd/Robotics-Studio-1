#!/usr/bin/env python3
"""Fast validation that does not require a ROS installation."""

from __future__ import annotations

import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REQUIRED = [
    "README.md",
    "src/embertrace_interfaces/package.xml",
    "src/embertrace_sim/package.xml",
    "src/embertrace_core/package.xml",
    "src/embertrace_ui/package.xml",
    "src/embertrace_bringup/package.xml",
    "src/embertrace_sim/worlds/embertrace_forest.sdf",
    "src/embertrace_bringup/launch/embertrace_demo.launch.py",
]


def fail(message: str) -> None:
    print(f"ERROR: {message}", file=sys.stderr)
    raise SystemExit(1)


for relative in REQUIRED:
    if not (ROOT / relative).is_file():
        fail(f"required file is missing: {relative}")

for path in sorted(ROOT.glob("src/*/package.xml")):
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        fail(f"invalid package XML in {path.relative_to(ROOT)}: {exc}")
    if root.tag != "package" or root.findtext("name") != path.parent.name:
        fail(f"package name/path mismatch in {path.relative_to(ROOT)}")

for pattern in ("src/**/*.sdf", "src/**/*.urdf", "src/**/*.xacro", "src/**/model.config"):
    for path in sorted(ROOT.glob(pattern)):
        try:
            ET.parse(path)
        except ET.ParseError as exc:
            fail(f"invalid XML in {path.relative_to(ROOT)}: {exc}")

for path in sorted(ROOT.glob("src/**/*.launch.py")):
    try:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    except SyntaxError as exc:
        fail(f"invalid Python syntax in {path.relative_to(ROOT)}: {exc}")

text_suffixes = {".md", ".txt", ".py", ".cpp", ".hpp", ".xml", ".yaml", ".yml", ".sdf", ".xacro"}
for path in ROOT.rglob("*"):
    if path.is_file() and path.suffix in text_suffixes:
        if path.resolve() == Path(__file__).resolve():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if "FindPackageShare('41068_ignition_bringup')" in text or "$(find 41068_ignition_bringup)" in text:
            fail(f"obsolete package reference remains in {path.relative_to(ROOT)}")

print(f"EmberTrace repository validation passed ({ROOT}).")
