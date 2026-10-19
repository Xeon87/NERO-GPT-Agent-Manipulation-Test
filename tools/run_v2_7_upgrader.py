#!/usr/bin/env python3
"""Run the V2-7 local upgrade payload from this experimental branch.

This wrapper also applies packaging-time compatibility fixes so the upgrader
accepts the actual V2-6 Egg Task Semantic Integration Agent layout.  The
semantic Agent does not necessarily contain the older literal headings
"State machine:" or "Contract rules:", so guide insertion falls back to the
stable "Operating policy:" / TOOLS anchors instead of aborting.
"""
from pathlib import Path

payload = Path(__file__).with_name("upgrade_v2_6_egg_to_v2_7.py")
text = payload.read_text(encoding="utf-8")

# Repair a packaging-only quote typo in the payload's generated README text.
text = text.replace(
    '        """# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    '        "# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    1,
)

# Compatibility hotfix for the real V2-6 Egg Semantic Agent.  Earlier public
# Agents had a literal "State machine:" / "Contract rules:" heading, while the
# Egg Semantic Integration Agent can omit both.  Preserve the preferred old
# anchors, but fall back to the stable operating-policy or TOOLS section.
old_guide_block = '''    # Insert the policy before the existing state-machine/contract section.\n    if "\\nState machine:\\n" in text:\n        text = text.replace("\\nState machine:\\n", "\\n" + AGENT_V27_GUIDE + "\\nState machine:\\n", 1)\n    elif "\\nContract rules:\\n" in text:\n        text = text.replace("\\nContract rules:\\n", "\\n" + AGENT_V27_GUIDE + "\\nContract rules:\\n", 1)\n    else:\n        raise RuntimeError("Could not find Agent guide insertion point")\n'''
new_guide_block = '''    # Insert the policy before the existing state-machine/contract section.\n    if "\\nState machine:\\n" in text:\n        text = text.replace("\\nState machine:\\n", "\\n" + AGENT_V27_GUIDE + "\\nState machine:\\n", 1)\n    elif "\\nContract rules:\\n" in text:\n        text = text.replace("\\nContract rules:\\n", "\\n" + AGENT_V27_GUIDE + "\\nContract rules:\\n", 1)\n    elif "Operating policy:\\n" in text:\n        text = text.replace("Operating policy:\\n", AGENT_V27_GUIDE + "\\nOperating policy:\\n", 1)\n    elif "TOOLS = [\\n" in text:\n        text = text.replace("TOOLS = [\\n", AGENT_V27_GUIDE + "\\nTOOLS = [\\n", 1)\n    else:\n        raise RuntimeError("Could not find a stable Agent guide insertion point")\n'''
if old_guide_block not in text:
    raise RuntimeError("Packaging hotfix could not locate the Agent guide patch block")
text = text.replace(old_guide_block, new_guide_block, 1)

# The Egg Semantic Agent may call this section "Operating policy:" or may have
# already folded those instructions into its main prompt.  Do not make a
# missing prose heading fatal; if present, prepend the V2-7 override.  If not,
# the V2-7 guide inserted above already contains the same mandatory sequence.
old_policy_block = '''    policy_marker = "Operating policy:\\n"\n    extra = (\n        "Operating policy:\\n"\n        "0. V2-7 EGG PICKUP OVERRIDE: for egg_1..egg_4 in the basket, after the mandatory observe_scene and aperture preparation, use plan_grasp -> move_to_pregrasp -> approach_planned_grasp before clamp_gripper. Do not repeatedly use legacy move_above/approach for egg pickup.\\n"\n    )\n    text = replace_once(text, policy_marker, extra, "Agent V2-7 operating policy")\n'''
new_policy_block = '''    policy_marker = "Operating policy:\\n"\n    extra = (\n        "Operating policy:\\n"\n        "0. V2-7 EGG PICKUP OVERRIDE: for egg_1..egg_4 in the basket, after the mandatory observe_scene and aperture preparation, use plan_grasp -> move_to_pregrasp -> approach_planned_grasp before clamp_gripper. Do not repeatedly use legacy move_above/approach for egg pickup.\\n"\n    )\n    if policy_marker in text:\n        text = text.replace(policy_marker, extra, 1)\n'''
if old_policy_block in text:
    text = text.replace(old_policy_block, new_policy_block, 1)

code = compile(text, str(payload), "exec")
ns = {
    "__name__": "__main__",
    "__file__": str(payload),
    "__package__": None,
}
exec(code, ns, ns)
