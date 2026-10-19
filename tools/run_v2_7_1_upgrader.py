#!/usr/bin/env python3
"""Run the V2-7.1 upgrade payload with packaging/runtime hardening.

This wrapper intentionally modifies only the *installer source text* before
execution.  It avoids making the large generated-code payload depend on the
exact Agent import name left by an older local runtime.
"""
from pathlib import Path

payload = Path(__file__).with_name("upgrade_v2_7_to_v2_7_1.py")
text = payload.read_text(encoding="utf-8")

# 1) A local V2-7 runtime may still carry the original V2-6 Agent import in the
# backend even though its launcher installs the V2-7 Agent filename.  Make the
# generated V2-7.1 backend self-contained in either case.
old_import_patch = (
    '    text = text.replace("from NERO_V2_7_Agent import", "from NERO_V2_7_1_Agent import")\n'
)
new_import_patch = (
    '    text = text.replace("from NERO_V2_7_Agent import", "from NERO_V2_7_1_Agent import")\n'
    '    text = text.replace("from NERO_V2_6_Agent import", "from NERO_V2_7_1_Agent import")\n'
)
if old_import_patch in text:
    text = text.replace(old_import_patch, new_import_patch, 1)

# 2) Once an egg is lifted, deterministic next-action guidance must point to the
# new placement planner.  This makes the normalized skill result consistent with
# the V2-7.1 Agent policy instead of nudging Luna back to the V2-6/V2-7 path.
return_patch = '    text = text.replace("NERO V2-7 unified runtime log", "NERO V2-7.1 unified runtime log")\n    return text\n'
return_replacement = (
    '    text = text.replace("NERO V2-7 unified runtime log", "NERO V2-7.1 unified runtime log")\n'
    '    text = text.replace(\'V24_STAGE_LIFTED: ["move_above_receptacle"]\', \'V24_STAGE_LIFTED: ["plan_placement"]\')\n'
    '    text = text.replace(\'recommended_next_actions=["move_above_receptacle"]\', \'recommended_next_actions=["plan_placement"]\')\n'
    '    return text\n'
)
if return_patch in text:
    text = text.replace(return_patch, return_replacement, 1)

# 3) Block the legacy egg placement motions deterministically even if a low-
# reasoning model ignores the prompt.  The old handlers remain available for
# non-egg/manual compatibility, but an active egg may only use the V2-7.1 path.
handler_anchor = '''        # If an egg was unexpectedly dropped after lift/placement, the old
        # V2-7 state could remain RECEPTACLE_HOVER_REACHED forever even though
'''
legacy_guard = '''        # Deterministic routing guard: the old receptacle descent is exactly the
        # path that collided with the egg holder in V2-7.  Do not rely on prompt
        # obedience alone; refuse those old motions for an active egg.
        if name in ("move_above_receptacle", "place_into_receptacle"):
            _legacy_obj = str(state_before.get("active_object") or "")
            if _legacy_obj.startswith("egg"):
                return _v24_precondition_failure(
                    name, args,
                    "V2-7.1 blocks legacy egg placement; use long-axis placement planning instead",
                    contract, state_before,
                    recommended_next_actions=["plan_placement"] if state_before.get("stage") == V24_STAGE_LIFTED else ["observe_scene", "plan_placement"],
                )
        if name == "open_gripper":
            _legacy_obj = str(state_before.get("active_object") or "")
            if _legacy_obj.startswith("egg") and state_before.get("stage") not in (V24_STAGE_RELEASED, V24_STAGE_VERIFIED):
                return _v24_precondition_failure(
                    name, args,
                    "V2-7.1 blocks direct egg release before guarded_insert_and_release reaches capture depth",
                    contract, state_before,
                    recommended_next_actions=["plan_placement"] if state_before.get("stage") == V24_STAGE_LIFTED else ["observe_scene"],
                )

'''
if handler_anchor in text and "V2-7.1 blocks legacy egg placement" not in text:
    text = text.replace(handler_anchor, legacy_guard + handler_anchor, 1)

# Compile the repaired installer itself before running it.
code = compile(text, str(payload), "exec")
ns = {
    "__name__": "__main__",
    "__file__": str(payload),
    "__package__": None,
}
exec(code, ns, ns)
