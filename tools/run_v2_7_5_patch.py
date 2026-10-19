#!/usr/bin/env python3
"""Compatibility shim for exact V2-7.2 generated launcher filenames."""
from pathlib import Path
import sys

import upgrade_v2_7_2_to_v2_7_5 as impl


def _patch_launcher_v272_exact(text: str) -> str:
    backend_old = "NERO_V2_7_1_Unified_Backend.py"
    agent_old = "NERO_V2_7_1_Agent.py"
    nb = text.count(backend_old)
    na = text.count(agent_old)
    if nb < 1 or na < 1:
        raise RuntimeError(
            f"Unexpected V2-7.2 launcher entry refs: backend={nb}, agent={na}"
        )
    text = text.replace(backend_old, impl.OUT_BACKEND)
    text = text.replace(agent_old, impl.OUT_AGENT)
    text = text.replace("NERO V2-7.2", "NERO V2-7.5")
    return text


impl.patch_launcher = _patch_launcher_v272_exact

if __name__ == "__main__":
    impl.main()

    # Packaging-only cleanup. V2-7.2 carries historical launchers forward when
    # its runtime tree is copied. They are not used by V2-7.5 and can only create
    # ambiguity for a human running the extracted ZIP, so keep exactly one entry.
    if "--output-dir" not in sys.argv:
        raise RuntimeError("V2-7.5 compatibility shim requires --output-dir")
    out = Path(sys.argv[sys.argv.index("--output-dir") + 1]).expanduser().resolve()
    for stale in (
        "start_nero_v2_7.sh",
        "start_nero_v2_7_1.sh",
        "start_nero_v2_7_2.sh",
        "start_nero_v2_7_3.sh",
        "start_nero_v2_7_4.sh",
        "start_nero_v2_7_4_1.sh",
        "start_nero_v2_7_4_2.sh",
    ):
        p = out / stale
        if p.is_file():
            p.unlink()

    expected = [
        impl.OUT_BACKEND,
        impl.OUT_AGENT,
        impl.CORE,
        impl.OUT_LAUNCHER,
        "PATH_CHECK.txt",
        "README_V2_7_5.md",
        "SHA256SUMS.txt",
    ]
    missing = [name for name in expected if not (out / name).is_file()]
    if missing:
        raise RuntimeError(f"V2-7.5 final package contract missing: {missing}")
    stale_left = [p.name for p in out.glob("start_nero_v2_*.sh") if p.name != impl.OUT_LAUNCHER]
    if stale_left:
        raise RuntimeError(f"V2-7.5 stale launcher cleanup failed: {stale_left}")
    print("[OK] V2-7.5 package has exactly one launcher:", impl.OUT_LAUNCHER)
