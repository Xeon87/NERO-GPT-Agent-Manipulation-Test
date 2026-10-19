#!/usr/bin/env python3
"""Compatibility shim for the actual generated V2-7.2 runtime.

V2-7.2 is an incremental hardening layer over V2-7.1 and intentionally keeps
its runtime Python filenames as NERO_V2_7_1_*.  It also inherits/version-rewrites
some log labels from earlier layers.  The generic V2-7.3 upgrader deliberately
uses strict anchors, so this shim normalizes only those known compatibility
variants before applying the real patch.
"""
import re
import upgrade_v2_7_2_to_v2_7_3 as impl

impl.BACKEND = "NERO_V2_7_1_Unified_Backend.py"
impl.AGENT = "NERO_V2_7_1_Agent.py"
impl.CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

_original_patch_backend = impl.patch_backend


def normalize_v272_backend_text(text: str) -> str:
    """Normalize known V2-7.2 inherited/versioned strings only.

    The V2-7.3 semantic patch expects the historical literal
    ``[V2-6 LATE SETTLE ACCEPT]``.  Depending on the exact V2-7.1 -> V2-7.2
    generation path, that same code can be labeled V2-7, V2-7.1, or V2-7.2.
    Normalize the label without touching control flow.
    """
    text, n = re.subn(
        r"\[V2-7(?:\.1|\.2)? LATE SETTLE ACCEPT\]",
        "[V2-6 LATE SETTLE ACCEPT]",
        text,
    )
    if n > 1:
        raise RuntimeError(
            f"V2-7.3 compatibility normalization found {n} late-settle markers; expected <=1"
        )
    return text


def compat_patch_backend(text: str) -> str:
    return _original_patch_backend(normalize_v272_backend_text(text))


impl.patch_backend = compat_patch_backend

if __name__ == "__main__":
    impl.main()
