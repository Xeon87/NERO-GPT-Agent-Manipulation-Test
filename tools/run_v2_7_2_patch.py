#!/usr/bin/env python3
"""Compatibility/hardening shim for the actual generated V2-7.1 runtime."""
import upgrade_v2_7_1_to_v2_7_2 as impl

impl.BACKEND = "NERO_V2_7_1_Unified_Backend.py"
impl.AGENT = "NERO_V2_7_1_Agent.py"
impl.CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

# Strengthen the visual-identity helper: enumerate every physical body whose
# name starts with obj_egg, not only the four expected names.  This makes a
# genuine fifth egg body a startup error instead of a visual ambiguity.
impl.HELPERS = impl.HELPERS.replace(
    "    found = []\n    for digit in range(1, 5):",
    "    all_egg_bodies = sorted(\n"
    "        str(b.get('name')) for b in root.findall('.//body')\n"
    "        if str(b.get('name', '')).startswith('obj_egg')\n"
    "    )\n"
    "    expected_egg_bodies = ['obj_egg1', 'obj_egg2', 'obj_egg3', 'obj_egg4']\n"
    "    if all_egg_bodies != expected_egg_bodies:\n"
    "        raise RuntimeError(\n"
    "            f'V2-7.2 physical egg body registry mismatch: expected={expected_egg_bodies}, actual={all_egg_bodies}'\n"
    "        )\n"
    "    found = []\n"
    "    for digit in range(1, 5):"
)
impl.HELPERS = impl.HELPERS.replace(
    "    print(f\"[V2-7.2 EGG IDENTITY] labelled bodies={found} count={len(found)} expected=4\", flush=True)\n"
    "    if len(found) != 4:\n"
    "        raise RuntimeError(f\"Expected exactly 4 named egg bodies, found {len(found)}: {found}\")\n",
    "    print(\n"
    "        f\"[V2-7.2 EGG IDENTITY] physical egg bodies={all_egg_bodies} | labelled={found} | count={len(all_egg_bodies)}\",\n"
    "        flush=True,\n"
    "    )\n"
    "    if found != expected_egg_bodies:\n"
    "        raise RuntimeError(f\"V2-7.2 egg label registry mismatch: {found}\")\n"
)

if __name__ == "__main__":
    impl.main()
