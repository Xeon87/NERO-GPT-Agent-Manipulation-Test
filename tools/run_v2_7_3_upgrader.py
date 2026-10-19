#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys

home = Path.home()
candidates = [
    home / "mujoco_ws" / "nero" / "NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation",
    Path.cwd() / "NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation",
]
src = next((p for p in candidates if p.is_dir()), None)
if src is None:
    print("ERROR: 找不到已验证的 V2-7.2 runtime。")
    print("Expected one of:")
    for p in candidates:
        print("  ", p)
    sys.exit(2)

# Actual generated V2-7.2 runtime intentionally keeps the V2-7.1 Python
# filenames.  Validate those real files before launching the compatibility shim.
required = [
    "NERO_V2_7_1_Unified_Backend.py",
    "NERO_V2_7_1_Agent.py",
    "NERO_V69_ReachableZone_BioIK_Planner_Core.py",
]
missing = [name for name in required if not (src / name).is_file()]
if missing:
    print("ERROR: V2-7.2 runtime directory exists but required files are missing:")
    for name in missing:
        print("  ", src / name)
    print("Directory contents:")
    for p in sorted(src.iterdir()):
        print("  ", p.name)
    sys.exit(3)

out = home / "mujoco_ws" / "nero" / "NERO_V2_7_3_Late_Settle_Task_Summary"
patcher = Path(__file__).resolve().with_name("run_v2_7_3_patch.py")

print("[INFO] Source V2-7.2:", src)
print("[INFO] Backend:", src / "NERO_V2_7_1_Unified_Backend.py")
print("[INFO] Agent:", src / "NERO_V2_7_1_Agent.py")
print("[INFO] Creating V2-7.3:", out)
subprocess.run(
    [sys.executable, str(patcher), str(src), "--output-dir", str(out)],
    check=True,
)
