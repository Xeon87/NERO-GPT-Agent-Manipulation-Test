#!/usr/bin/env python3
"""Compatibility shim for exact V2-7.2 generated launcher filenames."""
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
