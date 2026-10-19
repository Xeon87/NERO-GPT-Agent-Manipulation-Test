#!/usr/bin/env python3
from pathlib import Path
import subprocess
import sys

home = Path.home()
candidates = [
    home / "mujoco_ws" / "nero" / "NERO_V2_7_1_Placement_Aware_Egg_Handling",
    Path.cwd() / "NERO_V2_7_1_Placement_Aware_Egg_Handling",
]
src = next((p for p in candidates if p.is_dir()), None)
if src is None:
    print("ERROR: 找不到已验证的 V2-7.1 runtime。")
    print("Expected one of:")
    for p in candidates:
        print("  ", p)
    sys.exit(2)

out = home / "mujoco_ws" / "nero" / "NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation"
patcher = Path(__file__).resolve().with_name("run_v2_7_2_patch.py")

print("[INFO] Source V2-7.1:", src)
print("[INFO] Creating V2-7.2:", out)
subprocess.run(
    [sys.executable, str(patcher), str(src), "--output-dir", str(out)],
    check=True,
)
