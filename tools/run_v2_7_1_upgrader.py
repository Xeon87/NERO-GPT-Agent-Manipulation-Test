#!/usr/bin/env python3
"""Run the V2-7.1 upgrade payload with packaging/runtime hardening.

This wrapper intentionally repairs the installer source text before execution.
It lets the release remain a compact delta while refusing to silently build if
one of the validated V2-7.1 anchors is missing.
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
if old_import_patch not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: Agent import hardening")
text = text.replace(old_import_patch, new_import_patch, 1)

# 2) Once an egg is lifted, deterministic next-action guidance must point to the
# new placement planner.  This makes tool feedback agree with the Agent policy.
return_patch = '    text = text.replace("NERO V2-7 unified runtime log", "NERO V2-7.1 unified runtime log")\n    return text\n'
return_replacement = (
    '    text = text.replace("NERO V2-7 unified runtime log", "NERO V2-7.1 unified runtime log")\n'
    '    text = text.replace(\'V24_STAGE_LIFTED: ["move_above_receptacle"]\', \'V24_STAGE_LIFTED: ["plan_placement"]\')\n'
    '    text = text.replace(\'recommended_next_actions=["move_above_receptacle"]\', \'recommended_next_actions=["plan_placement"]\')\n'
    '    return text\n'
)
if return_patch not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: backend return hardening")
text = text.replace(return_patch, return_replacement, 1)

# 3) Block the legacy egg placement motions deterministically even if a low-
# reasoning model ignores the prompt.  The old handlers remain for non-egg or
# manual compatibility, but an active egg may only use the V2-7.1 path.
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
if handler_anchor not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: legacy placement guard")
text = text.replace(handler_anchor, legacy_guard + handler_anchor, 1)

# 4) First V2-7.1 experiment uses a shallow 8 mm capture depth.  The egg is
# geometrically captured by the cup before release, while the fingers remain
# farther from the holder walls/posts than in the original 14 mm draft.
if "V271_CAPTURE_DEPTH_M = 0.014" not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: capture depth")
text = text.replace("V271_CAPTURE_DEPTH_M = 0.014", "V271_CAPTURE_DEPTH_M = 0.008", 1)

# 5) Gentle release order matters.  At low height, open only enough to detach
# the egg.  Retreat the partially-open gripper vertically FIRST.  Only after the
# gripper is clear of the holder do we command the wide opening.  This avoids
# fanning the fingers outward into the cup posts immediately after insertion.
old_release = '''            time.sleep(0.20)
            gripper_target_mm.value = V271_RELEASE_FULL_OPEN_MM
            deadline = time.time() + 1.20
            while time.time() < deadline:
                if float(gripper_actual_mm.value) >= 80.0 and not bool(hold_active_shared.value):
                    break
                time.sleep(0.04)
            time.sleep(0.55)

            hold_after_release = bool(hold_active_shared.value)
            opening_after = float(gripper_actual_mm.value)
            obj_released, _ = skill_core.object_pose(obj)
            preliminary_location = llm_classify_object_location(obj_released)

            # Retreat the empty gripper straight upward after release.
            retreat_ok = False
            retreat_error = None
            try:
                tcp_r0, q_r0 = skill_core.current_tcp_pose()
                target_r = [float(tcp_r0[0]), float(tcp_r0[1]), float(tcp_r0[2]) + V271_RETREAT_M]
                def _do_retreat():
                    traj = skill_core._cartesian_approach_plan(target_r, q_r0)
                    skill_core._execute_trajectory(traj)
                    return True
                _vr, retreat_ok, _wr = _v24_run_motion(
                    f"v271_post_release_retreat({obj},{slot_name})",
                    _do_retreat,
                    target_r, 0.012, False,
                )
                warnings.extend(_wr)
            except Exception as _exc:
                retreat_error = str(_exc)
                warnings.append(f"Post-release vertical retreat raised: {_exc}")

            time.sleep(0.35)
            obj_final, _ = skill_core.object_pose(obj)
            final_location = llm_classify_object_location(obj_final)
            release_ok = bool((not hold_after_release) and opening_after >= 80.0)
'''
new_release = '''            time.sleep(0.25)

            # At capture depth, only loosen enough to detach the egg.  Leaving
            # the fingers modestly open keeps their lateral swept volume small
            # while they are still close to the holder posts.
            hold_after_release = bool(hold_active_shared.value)
            opening_after_stage1 = float(gripper_actual_mm.value)
            obj_released, _ = skill_core.object_pose(obj)
            preliminary_location = llm_classify_object_location(obj_released)

            # Retreat the partially-open gripper straight upward BEFORE asking
            # for a wide opening.  This is the low-clearance phase.
            retreat_ok = False
            retreat_error = None
            try:
                tcp_r0, q_r0 = skill_core.current_tcp_pose()
                target_r = [float(tcp_r0[0]), float(tcp_r0[1]), float(tcp_r0[2]) + V271_RETREAT_M]
                def _do_retreat():
                    traj = skill_core._cartesian_approach_plan(target_r, q_r0)
                    skill_core._execute_trajectory(traj)
                    return True
                _vr, retreat_ok, _wr = _v24_run_motion(
                    f"v271_post_release_retreat({obj},{slot_name})",
                    _do_retreat,
                    target_r, 0.012, False,
                )
                warnings.extend(_wr)
            except Exception as _exc:
                retreat_error = str(_exc)
                warnings.append(f"Post-release vertical retreat raised: {_exc}")

            # Wide opening is safe only after the fingers have cleared the cup.
            if retreat_ok:
                gripper_target_mm.value = V271_RELEASE_FULL_OPEN_MM
                deadline = time.time() + 1.20
                while time.time() < deadline:
                    if float(gripper_actual_mm.value) >= 80.0:
                        break
                    time.sleep(0.04)
            time.sleep(0.35)
            opening_after = float(gripper_actual_mm.value)
            obj_final, _ = skill_core.object_pose(obj)
            final_location = llm_classify_object_location(obj_final)
            stage1_opened_enough = opening_after_stage1 >= min(100.0, opening0 + 5.0)
            release_ok = bool((not hold_after_release) and stage1_opened_enough and retreat_ok)
'''
if old_release not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: staged release / retreat order")
text = text.replace(old_release, new_release, 1)

# Extend the generated verification payload so the log makes the release order
# directly inspectable during the first experiment.
verify_anchor = '                "opening_after_release_mm": round(opening_after, 2),\n'
verify_replacement = (
    '                "opening_after_stage1_mm": round(opening_after_stage1, 2),\n'
    '                "opening_after_release_mm": round(opening_after, 2),\n'
    '                "wide_open_command_after_retreat": bool(retreat_ok),\n'
)
if verify_anchor not in text:
    raise RuntimeError("V2-7.1 packaging anchor missing: release verification fields")
text = text.replace(verify_anchor, verify_replacement, 1)

# Compile the repaired installer itself before running it.  The installer then
# py_compile-checks the generated Backend and Agent and bash -n checks launcher.
code = compile(text, str(payload), "exec")
ns = {
    "__name__": "__main__",
    "__file__": str(payload),
    "__package__": None,
}
exec(code, ns, ns)
