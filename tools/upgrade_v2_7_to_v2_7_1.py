#!/usr/bin/env python3
"""Upgrade a validated local NERO V2-7 runtime to V2-7.1.

NERO V2-7.1 — Placement-Aware Egg Handling

Small-step scope only:
- V69 motion core remains byte-for-byte frozen.
- V2-7 constraint-aware egg grasp planning remains intact.
- Adds bystander-egg disturbance guard before clamp.
- Adds placement planning based on the egg LONG axis and egg-holder slot axis.
- Adds safe-height held-object reorientation with fixed TCP position; BioIK decides
  the joint distribution, so wrist J6/J7 are encouraged by current-pose seeding
  but are NOT hard-coded.
- Adds precise slot-entry alignment.
- Adds slow segmented axial insertion to a pre-release capture depth.
- Adds staged/slow release, vertical retreat, and final existing slot verify.
- Adds automatic logical recovery after an unexpected grasp loss.
- Adds a Copy Full Log button above the persistent GUI log.
- Does NOT model egg breaking, tactile damage, push/roll/nudge, or a new general
  skill architecture.

Expected source folder:
  ~/mujoco_ws/nero/NERO_V2_7_Constraint_Aware_Grasp_Planning

Creates:
  ~/mujoco_ws/nero/NERO_V2_7_1_Placement_Aware_Egg_Handling
"""

from __future__ import annotations

import argparse
import hashlib
import py_compile
import shutil
import stat
import subprocess
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"


BACKEND_V271_HELPERS = r'''

# ============================================================
# NERO V2-7.1 — Placement-Aware Egg Handling
# ============================================================
# V69 remains frozen.  These helpers live only in the semantic/task wrapper.
#
# Physical idea:
#   1) grasp the egg across its SHORT axis (V2-7)
#   2) lift
#   3) move to a safe orientation workspace
#   4) rotate the HELD egg so its LONG axis aligns with the slot entry axis
#   5) move precisely above the slot
#   6) descend slowly along that long axis only to a capture depth
#   7) release before the fingers have to enter the egg-holder walls
#   8) retreat vertically and let verify_object_in_slot decide success
# ============================================================

V271_SLOT_CENTERS_M = {
    "slot_1": (-0.510, 0.123, 0.043),
    "slot_2": (-0.450, 0.123, 0.043),
    "slot_3": (-0.390, 0.123, 0.043),
    "slot_4": (-0.330, 0.123, 0.043),
    "slot_5": (-0.510, 0.187, 0.043),
    "slot_6": (-0.450, 0.187, 0.043),
    "slot_7": (-0.390, 0.187, 0.043),
    "slot_8": (-0.330, 0.187, 0.043),
}
V271_SLOT_ENTRY_AXIS = (0.0, 0.0, -1.0)
V271_HOLDER_RIM_TOP_Z_M = 0.044
V271_SAFE_ORIENTATION_XY_M = (-0.4225, 0.0000)
V271_SAFE_ORIENTATION_MIN_Z_M = 0.245
V271_SAFE_ORIENTATION_EXTRA_Z_M = 0.085
V271_ENTRY_ABOVE_RELEASE_M = 0.055
V271_CAPTURE_DEPTH_M = 0.014
V271_SLOT_XY_TOL_M = 0.007
V271_SLOT_XY_ABORT_M = 0.012
V271_AXIS_TOL_DEG = 10.0
V271_INSERT_AXIS_ABORT_DEG = 12.0
V271_BYSTANDER_MOVE_TOL_M = 0.003
V271_RELATIVE_DISTANCE_DRIFT_TOL_M = 0.025
V271_INSERT_STEP_M = 0.010
V271_INSERT_FINE_STEP_M = 0.005
V271_RELEASE_STAGE_OPEN_MM = 10.0
V271_RELEASE_FULL_OPEN_MM = 90.0
V271_RETREAT_M = 0.080

V271_PLACEMENT_RUNTIME = {
    "plans": {},
    "oriented": {},
    "slot_entry": {},
    "released": {},
}
V271_BYSTANDER_BASELINE = {}


def _v271_reset_placement_runtime(object_names=()):
    V271_PLACEMENT_RUNTIME["plans"].clear()
    V271_PLACEMENT_RUNTIME["oriented"].clear()
    V271_PLACEMENT_RUNTIME["slot_entry"].clear()
    V271_PLACEMENT_RUNTIME["released"].clear()
    V271_BYSTANDER_BASELINE.clear()
    for _name in object_names:
        V271_BYSTANDER_BASELINE[str(_name)] = {}


def _v271_public_runtime_snapshot():
    def _safe(v):
        if isinstance(v, dict):
            return {str(k): _safe(x) for k, x in v.items()}
        if isinstance(v, (list, tuple)):
            return [_safe(x) for x in v]
        if isinstance(v, set):
            return sorted(str(x) for x in v)
        return v
    return {
        "plans": _safe(V271_PLACEMENT_RUNTIME.get("plans", {})),
        "oriented": _safe(V271_PLACEMENT_RUNTIME.get("oriented", {})),
        "slot_entry": _safe(V271_PLACEMENT_RUNTIME.get("slot_entry", {})),
        "released": _safe(V271_PLACEMENT_RUNTIME.get("released", {})),
    }


def _v271_dot(a, b):
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def _v271_cross(a, b):
    return [
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    ]


def _v271_norm(v):
    return math.sqrt(sum(float(x) * float(x) for x in v))


def _v271_unit(v):
    n = _v271_norm(v)
    if n < 1e-12:
        return [0.0, 0.0, 1.0]
    return [float(x) / n for x in v]


def _v271_quat_conjugate(q):
    x, y, z, w = _v27_quat_normalize(q)
    return [-x, -y, -z, w]


def _v271_quat_from_two_vectors(a, b):
    """World-frame quaternion that rotates vector a onto vector b."""
    va = _v271_unit(a)
    vb = _v271_unit(b)
    d = max(-1.0, min(1.0, _v271_dot(va, vb)))
    if d > 1.0 - 1e-10:
        return [0.0, 0.0, 0.0, 1.0]
    if d < -1.0 + 1e-10:
        # 180 deg: choose any stable axis perpendicular to a.
        ref = [1.0, 0.0, 0.0] if abs(va[0]) < 0.8 else [0.0, 1.0, 0.0]
        axis = _v271_unit(_v271_cross(va, ref))
        return [axis[0], axis[1], axis[2], 0.0]
    c = _v271_cross(va, vb)
    q = [c[0], c[1], c[2], 1.0 + d]
    return _v27_quat_normalize(q)


def _v271_slerp(q0, q1, t):
    qa = _v27_quat_normalize(q0)
    qb = _v27_quat_normalize(q1)
    dot = sum(float(qa[i]) * float(qb[i]) for i in range(4))
    if dot < 0.0:
        qb = [-float(x) for x in qb]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        q = [(1.0 - float(t)) * qa[i] + float(t) * qb[i] for i in range(4)]
        return _v27_quat_normalize(q)
    theta = math.acos(dot)
    s = math.sin(theta)
    a = math.sin((1.0 - float(t)) * theta) / s
    b = math.sin(float(t) * theta) / s
    return _v27_quat_normalize([a * qa[i] + b * qb[i] for i in range(4)])


def _v271_axis_line_error_deg(axis, target_axis=V271_SLOT_ENTRY_AXIS):
    a = _v271_unit(axis)
    b = _v271_unit(target_axis)
    # Egg long axis is an undirected line: +axis and -axis are equivalent.
    d = abs(max(-1.0, min(1.0, _v271_dot(a, b))))
    return math.degrees(math.acos(d))


def _v271_target_tcp_quat_for_slot(obj_quat, tcp_quat):
    axis_info = _v27_egg_major_axis_from_quat(obj_quat)
    current_axis = [float(v) for v in axis_info["world_axis"]]
    slot_axis = [float(v) for v in V271_SLOT_ENTRY_AXIS]
    # Choose +/- slot axis requiring the smaller rotation because the egg long
    # axis is a line, not a directed arrow.
    target_axis = slot_axis if _v271_dot(current_axis, slot_axis) >= 0.0 else [-v for v in slot_axis]
    q_delta = _v271_quat_from_two_vectors(current_axis, target_axis)
    target_tcp_q = _v27_quat_normalize(_v27_quat_multiply(q_delta, tcp_quat))
    return {
        "current_axis": current_axis,
        "target_axis": target_axis,
        "axis_error_before_deg": _v271_axis_line_error_deg(current_axis, slot_axis),
        "target_tcp_quat_xyzw": target_tcp_q,
    }


def _v271_slot(slot_name):
    return V271_SLOT_CENTERS_M.get(str(slot_name))


def _v271_egg_half_length_m(geom):
    try:
        size = geom.get("approx_size_mm", [44.0, 43.0, 62.0])
        return max(0.020, float(size[2]) / 2000.0)
    except Exception:
        return 0.031


def _v271_bystander_moves(skill_core, active_obj):
    baseline = V271_BYSTANDER_BASELINE.get(str(active_obj), {})
    moved = []
    for other, p0 in baseline.items():
        try:
            p1, _ = skill_core.object_pose(other)
            d = math.sqrt(sum((float(p1[i]) - float(p0[i])) ** 2 for i in range(3)))
            if d > V271_BYSTANDER_MOVE_TOL_M:
                moved.append({"object_name": other, "motion_mm": round(d * 1000.0, 2)})
        except Exception:
            pass
    moved.sort(key=lambda x: -float(x["motion_mm"]))
    return moved
'''


BACKEND_V271_CONTRACTS = r'''
            "plan_placement": {
                "purpose": f"Plan long-axis placement of held {obj} into an egg-holder slot without moving the robot.",
                "preconditions": ["state=LIFTED", "hold_active=true", "target egg-holder slot exists and is empty"],
                "expected_effects": [
                    "measure held egg long-axis orientation",
                    "compute the smaller rotation that aligns the egg long axis with the slot entry axis",
                    "choose a safe mid-workspace orientation pose",
                    "compute slot-entry and pre-release capture depths",
                ],
                "expected_non_effects": ["robot does not move", "egg does not move", "gripper does not open"],
                "success_criteria": ["valid slot and held egg", "safe orientation/entry/release targets produced"],
            },
            "orient_for_slot": {
                "purpose": f"Move held {obj} to a safe orientation workspace, then reorient it so its LONG axis aligns with the egg-holder slot axis.",
                "preconditions": ["plan_placement succeeded", "hold_active=true", "state=LIFTED"],
                "expected_effects": [
                    "transport the held egg to a high obstacle-free waypoint",
                    "rotate at approximately fixed TCP XYZ in 1-3 bounded orientation steps",
                    "use current-pose-seeded BioIK; wrist-dominant motion is preferred but J6/J7 are not hard-coded",
                ],
                "expected_non_effects": ["egg must not be released", "orientation change is NOT performed at the slot mouth"],
                "success_criteria": ["egg long-axis error <= 10 deg", "hold remains active", "no safety trip"],
            },
            "move_to_slot_entry": {
                "purpose": f"Move vertically oriented held {obj} to a precise hover/capture entry above the selected slot.",
                "preconditions": ["state=PLACEMENT_ORIENTED", "hold_active=true", "egg long axis aligned to slot axis"],
                "expected_effects": ["egg center aligns to slot XY while orientation is preserved"],
                "expected_non_effects": ["no release", "no large wrist reorientation near the holder"],
                "success_criteria": ["egg center XY error <= 7 mm", "axis error <= 10 deg", "hold active", "no safety trip"],
            },
            "guarded_insert_and_release": {
                "purpose": f"Slowly insert held {obj} along its LONG axis to a safe capture depth, then gently release and retreat.",
                "preconditions": ["state=SLOT_ENTRY_ALIGNED", "hold_active=true", "XY and long-axis alignment verified"],
                "expected_effects": [
                    "segmented 10 mm then 5 mm Cartesian descent along slot axis",
                    "continuous XY/orientation/hold/safety checks after every segment",
                    "stop above the final slot center so the fingers do not need to enter the holder walls",
                    "staged opening/release, then vertical gripper retreat",
                ],
                "expected_non_effects": ["no high-speed plunge", "no late large wrist rotation", "this skill does not replace final verify_object_in_slot"],
                "success_criteria": ["capture depth reached safely", "grasp released", "gripper retreats vertically", "final placement is left to verify_object_in_slot"],
            },
'''


BACKEND_V271_HANDLERS = r'''
        # ------------------------------------------------------------
        # NERO V2-7.1 Placement-Aware Egg Handling
        # ------------------------------------------------------------

        # If an egg was unexpectedly dropped after lift/placement, the old
        # V2-7 state could remain RECEPTACLE_HOVER_REACHED forever even though
        # hold_active=false.  On the next observation create a clean semantic
        # boundary so a fresh pickup can begin.
        if name == "observe_scene":
            _stage_now = str(state_before.get("stage") or "")
            if (
                not bool(hold_active_shared.value)
                and _stage_now in (
                    "LIFTED",
                    "RECEPTACLE_HOVER_REACHED",
                    "SLOT_PLACE_REACHED",
                    "PLACEMENT_ORIENTED",
                    "SLOT_ENTRY_ALIGNED",
                )
            ):
                skill_core.emit(
                    f"[V2-7.1 HOLD-LOSS STATE RECOVERY] stage={_stage_now} but hold_active=false -> fresh OBSERVE episode"
                )
                _v24_reset_skill_state("V2-7.1 unexpected hold loss", clear_adaptation=False)
                state_before = _v24_state_snapshot()

        # Before the existing clamp skill is allowed to close, verify that the
        # candidate approach did not shove a NON-target egg.  This directly
        # catches the V2-7 failure mode where a finger visibly poked another egg
        # but the active target still looked valid.
        if name == "clamp_gripper":
            _obj_guard = str(state_before.get("active_object") or "")
            if _obj_guard.startswith("egg") and state_before.get("stage") == V24_STAGE_GRASP_POSE:
                _moved = _v271_bystander_moves(skill_core, _obj_guard)
                if _moved:
                    _cid = V27_GRASP_RUNTIME.get("selected", {}).get(_obj_guard)
                    if _cid:
                        V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(_obj_guard, set()).add(str(_cid))
                    _retreat_ok = False
                    try:
                        if int(safety_state.value) == 0:
                            skill_core.bio_lift(0.090)
                            _retreat_ok = True
                    except Exception as _exc:
                        skill_core.emit(f"[V2-7.1 BYSTANDER RETREAT WARN] {_exc}")
                    _v24_set_state(V24_STAGE_OBSERVED, active_object=None, target_bin=None, last_tool=name, last_ok=False)
                    _verification = {
                        "reason": "BYSTANDER_EGG_DISTURBED",
                        "active_object": _obj_guard,
                        "moved_bystanders": _moved,
                        "threshold_mm": round(V271_BYSTANDER_MOVE_TOL_M * 1000.0, 1),
                        "candidate_marked_tried": bool(_cid),
                        "retreat_attempted": True,
                        "retreat_ok": bool(_retreat_ok),
                    }
                    return _v24_result(
                        name, False, contract, state_before, _verification,
                        ["A non-target egg moved during the pre-clamp approach. Do not clamp or lift; choose another grasp candidate."],
                        {"elapsed_s": round(time.perf_counter()-t0, 3)},
                        next_stage=None,
                        recommended_next_actions=["observe_scene", "plan_grasp"],
                    )

        if name == "plan_placement":
            obj = str(args.get("object_name", state_before.get("active_object") or ""))
            receptacle = str(args.get("receptacle_name", "egg_holder"))
            slot_name = str(args.get("slot_name", ""))
            slot = _v271_slot(slot_name)
            if state_before.get("stage") != V24_STAGE_LIFTED or state_before.get("active_object") != obj:
                return _v24_precondition_failure(
                    name, args, f"requires LIFTED with same active object; current={state_before}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if not bool(hold_active_shared.value):
                return _v24_precondition_failure(
                    name, args, "hold_active=false; cannot plan placement for an unheld egg",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if receptacle != "egg_holder" or slot is None:
                return _v24_precondition_failure(
                    name, args, f"invalid egg-holder destination: receptacle={receptacle} slot={slot_name}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            try:
                _scene = llm_scene_snapshot()
                _slot_scene = _scene.get("receptacles", {}).get("egg_holder", {}).get("slots", {}).get(slot_name, {})
                if bool(_slot_scene.get("occupied")):
                    return _v24_precondition_failure(
                        name, args, f"{slot_name} is already occupied by {_slot_scene.get('occupied_by')}",
                        contract, state_before, recommended_next_actions=["observe_scene", "choose_empty_slot"],
                    )
            except Exception:
                pass

            obj_xyz, obj_q = skill_core.object_pose(obj)
            tcp_xyz, tcp_q = skill_core.current_tcp_pose()
            orient = _v271_target_tcp_quat_for_slot(obj_q, tcp_q)
            geom = dict(V24_OBJECT_GEOMETRY.get(obj, {}))
            half_len = _v271_egg_half_length_m(geom)
            release_obj_z = max(
                float(slot[2]) + 0.010,
                V271_HOLDER_RIM_TOP_Z_M + half_len - V271_CAPTURE_DEPTH_M,
            )
            entry_obj_z = release_obj_z + V271_ENTRY_ABOVE_RELEASE_M
            safe_z = max(
                V271_SAFE_ORIENTATION_MIN_Z_M,
                float(tcp_xyz[2]) + V271_SAFE_ORIENTATION_EXTRA_Z_M,
            )
            safe_z = min(0.340, safe_z)
            safe_xyz = [V271_SAFE_ORIENTATION_XY_M[0], V271_SAFE_ORIENTATION_XY_M[1], safe_z]

            plan = {
                "object_name": obj,
                "receptacle_name": receptacle,
                "slot_name": slot_name,
                "slot_center_xyz_m": [float(v) for v in slot],
                "slot_entry_axis": [float(v) for v in V271_SLOT_ENTRY_AXIS],
                "object_xyz_at_plan_m": [round(float(v), 6) for v in obj_xyz],
                "object_quat_at_plan_xyzw": [round(float(v), 8) for v in obj_q],
                "tcp_xyz_at_plan_m": [round(float(v), 6) for v in tcp_xyz],
                "egg_major_axis_world": [round(float(v), 6) for v in orient["current_axis"]],
                "axis_error_before_deg": round(float(orient["axis_error_before_deg"]), 2),
                "target_axis_world": [round(float(v), 6) for v in orient["target_axis"]],
                "safe_orientation_xyz_m": [round(float(v), 6) for v in safe_xyz],
                "entry_object_center_z_m": round(float(entry_obj_z), 6),
                "pre_release_object_center_z_m": round(float(release_obj_z), 6),
                "egg_half_length_mm": round(half_len * 1000.0, 2),
                "capture_depth_mm": round(V271_CAPTURE_DEPTH_M * 1000.0, 1),
                "policy": "short-axis grasp -> safe-height long-axis reorientation -> precise slot entry -> slow axial insertion -> staged release",
            }
            V271_PLACEMENT_RUNTIME["plans"][obj] = plan
            V271_PLACEMENT_RUNTIME["oriented"].pop(obj, None)
            V271_PLACEMENT_RUNTIME["slot_entry"].pop(obj, None)
            V271_PLACEMENT_RUNTIME["released"].pop(obj, None)
            skill_core.emit(
                f"[V2-7.1 PLACEMENT PLAN] {obj} -> {slot_name} | long-axis error={plan['axis_error_before_deg']:.1f}deg | "
                f"safe_orient_z={safe_z*1000.0:.0f}mm | entry_center_z={entry_obj_z*1000.0:.0f}mm | "
                f"pre_release_center_z={release_obj_z*1000.0:.0f}mm"
            )
            verification = {
                "planning_only": True,
                "physical_motion_expected": False,
                "axis_error_before_deg": plan["axis_error_before_deg"],
                "safe_orientation_xyz_m": plan["safe_orientation_xyz_m"],
                "entry_object_center_z_m": plan["entry_object_center_z_m"],
                "pre_release_object_center_z_m": plan["pre_release_object_center_z_m"],
                "wrist_dominant_intent": True,
                "hard_coded_joint6_joint7": False,
            }
            return _v24_result(
                name, True, contract, state_before, verification, [],
                {"placement_plan": plan, "elapsed_s": round(time.perf_counter()-t0, 3)},
                next_stage=None,
                recommended_next_actions=["orient_for_slot"],
            )

        if name == "orient_for_slot":
            obj = str(args.get("object_name", state_before.get("active_object") or ""))
            slot_name = str(args.get("slot_name", ""))
            plan = V271_PLACEMENT_RUNTIME.get("plans", {}).get(obj)
            if not plan or plan.get("slot_name") != slot_name:
                return _v24_precondition_failure(
                    name, args, "no matching fresh placement plan; call plan_placement first",
                    contract, state_before, recommended_next_actions=["plan_placement"],
                )
            if state_before.get("stage") != V24_STAGE_LIFTED or state_before.get("active_object") != obj:
                return _v24_precondition_failure(
                    name, args, f"requires LIFTED with {obj}; current={state_before}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if not bool(hold_active_shared.value):
                return _v24_precondition_failure(name, args, "hold_active=false", contract, state_before, recommended_next_actions=["observe_scene"])

            llm_require_arm_controller()
            safety0 = int(safety_trip_count.value)
            obj0, _ = skill_core.object_pose(obj)
            tcp0, q0 = skill_core.current_tcp_pose()
            rel_dist0 = _v24_xyz_distance(obj0, tcp0)
            safe_xyz = [float(v) for v in plan["safe_orientation_xyz_m"]]

            # Step A: move the still-held egg to a high, open workspace while
            # preserving current grasp orientation.  Do NOT rotate near the slot.
            verification_a, ok_a, warn_a = _v24_run_motion(
                f"v271_safe_orientation_waypoint({obj},{slot_name})",
                lambda: skill_core.move_to_pose(
                    safe_xyz, q0,
                    velocity_scale=0.12,
                    accel_scale=0.12,
                    orientation_tolerance=0.12,
                    position_tolerance=0.010,
                ),
                safe_xyz, 0.015, True,
            )
            if (not ok_a) or (not bool(hold_active_shared.value)):
                verification_a["reason"] = verification_a.get("reason", "SAFE_ORIENTATION_WAYPOINT_FAILED")
                verification_a["hold_active"] = bool(hold_active_shared.value)
                return _v24_result(
                    name, False, contract, state_before, verification_a, warn_a,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            # Step B: at essentially fixed XYZ, rotate the complete held-object
            # frame.  BioIK uses the current physical state as seed; fixed XYZ +
            # minimum-displacement regularization tends to make this wrist-heavy,
            # but we intentionally do NOT hard-code J6/J7 angles.
            obj1, obj_q1 = skill_core.object_pose(obj)
            tcp1, tcp_q1 = skill_core.current_tcp_pose()
            orient = _v271_target_tcp_quat_for_slot(obj_q1, tcp_q1)
            q_target = [float(v) for v in orient["target_tcp_quat_xyzw"]]
            rotation_deg = _v27_quat_rotation_angle_deg(tcp_q1, q_target)
            n_steps = 1 if rotation_deg <= 35.0 else (2 if rotation_deg <= 70.0 else 3)
            warnings = list(warn_a)
            step_records = []
            ok_steps = True
            for idx in range(1, n_steps + 1):
                frac = float(idx) / float(n_steps)
                q_step = _v271_slerp(tcp_q1, q_target, frac)
                verification_s, ok_s, warn_s = _v24_run_motion(
                    f"v271_orient_step_{idx}of{n_steps}({obj})",
                    lambda q_step=q_step: skill_core.move_to_pose(
                        safe_xyz, q_step,
                        velocity_scale=0.07,
                        accel_scale=0.07,
                        orientation_tolerance=0.10,
                        position_tolerance=0.008,
                    ),
                    safe_xyz, 0.012, True,
                )
                warnings.extend(warn_s)
                step_records.append({
                    "step": idx,
                    "of": n_steps,
                    "tcp_error_mm": verification_s.get("tcp_error_mm"),
                    "cartesian_ok": bool(ok_s),
                    "hold_active": bool(hold_active_shared.value),
                })
                if (not ok_s) or (not bool(hold_active_shared.value)) or int(safety_trip_count.value) != safety0:
                    ok_steps = False
                    break

            obj2, obj_q2 = skill_core.object_pose(obj)
            tcp2, _ = skill_core.current_tcp_pose()
            axis2 = _v27_egg_major_axis_from_quat(obj_q2)["world_axis"]
            axis_error = _v271_axis_line_error_deg(axis2)
            rel_dist2 = _v24_xyz_distance(obj2, tcp2)
            rel_drift = abs(rel_dist2 - rel_dist0)
            ok = bool(
                ok_steps
                and bool(hold_active_shared.value)
                and int(safety_trip_count.value) == safety0
                and axis_error <= V271_AXIS_TOL_DEG
                and rel_drift <= V271_RELATIVE_DISTANCE_DRIFT_TOL_M
            )
            verification = {
                "safe_orientation_xyz_m": [round(float(v), 5) for v in safe_xyz],
                "requested_tcp_rotation_deg": round(rotation_deg, 2),
                "orientation_steps": step_records,
                "egg_major_axis_after": [round(float(v), 5) for v in axis2],
                "axis_error_to_slot_deg": round(axis_error, 2),
                "axis_tolerance_deg": V271_AXIS_TOL_DEG,
                "object_tcp_distance_drift_mm": round(rel_drift * 1000.0, 2),
                "hold_active": bool(hold_active_shared.value),
                "safety_trip_before": safety0,
                "safety_trip_after": int(safety_trip_count.value),
                "fixed_xyz_reorientation": True,
                "wrist_dominant_intent": True,
                "joint_solution": "BioIK current-state seed; J6/J7 preferred by minimum displacement, not hard-coded",
            }
            if not ok:
                verification["reason"] = (
                    "GRASP_LOST_DURING_REORIENTATION" if not bool(hold_active_shared.value)
                    else "LONG_AXIS_REORIENTATION_FAILED"
                )
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            V271_PLACEMENT_RUNTIME["oriented"][obj] = {
                "slot_name": slot_name,
                "axis_error_deg": round(axis_error, 2),
                "tcp_xyz_m": [round(float(v), 6) for v in tcp2],
            }
            skill_core.emit(
                f"[V2-7.1 ORIENT OK] {obj} | long-axis -> {slot_name} entry axis | error={axis_error:.1f}deg | "
                f"rotation={rotation_deg:.1f}deg in {n_steps} step(s) | hold=true"
            )
            return _v24_result(
                name, True, contract, state_before, verification, warnings,
                {"elapsed_s": round(time.perf_counter()-t0, 3)},
                "PLACEMENT_ORIENTED", obj,
                recommended_next_actions=["move_to_slot_entry"],
            )

        if name == "move_to_slot_entry":
            obj = str(args.get("object_name", state_before.get("active_object") or ""))
            slot_name = str(args.get("slot_name", ""))
            plan = V271_PLACEMENT_RUNTIME.get("plans", {}).get(obj)
            slot = _v271_slot(slot_name)
            if not plan or plan.get("slot_name") != slot_name or slot is None:
                return _v24_precondition_failure(name, args, "no matching placement plan", contract, state_before, recommended_next_actions=["plan_placement"])
            if state_before.get("stage") != "PLACEMENT_ORIENTED" or state_before.get("active_object") != obj:
                return _v24_precondition_failure(
                    name, args, f"requires PLACEMENT_ORIENTED for {obj}; current={state_before}",
                    contract, state_before, recommended_next_actions=["orient_for_slot"],
                )
            if not bool(hold_active_shared.value):
                return _v24_precondition_failure(name, args, "hold_active=false", contract, state_before, recommended_next_actions=["observe_scene"])

            obj0, obj_q0 = skill_core.object_pose(obj)
            tcp0, tcp_q0 = skill_core.current_tcp_pose()
            axis0 = _v27_egg_major_axis_from_quat(obj_q0)["world_axis"]
            axis_error0 = _v271_axis_line_error_deg(axis0)
            if axis_error0 > V271_AXIS_TOL_DEG:
                return _v24_precondition_failure(
                    name, args, f"egg long-axis alignment degraded to {axis_error0:.1f}deg; reorient before approaching holder",
                    contract, state_before,
                    physical_verification={"axis_error_deg": round(axis_error0,2)},
                    recommended_next_actions=["plan_placement", "orient_for_slot"],
                )

            llm_require_arm_controller()
            rel0 = [float(obj0[i]) - float(tcp0[i]) for i in range(3)]
            desired_obj = [float(slot[0]), float(slot[1]), float(plan["entry_object_center_z_m"])]
            target_tcp = [desired_obj[i] - rel0[i] for i in range(3)]
            verification1, ok1, warnings = _v24_run_motion(
                f"v271_slot_entry({obj},{slot_name})",
                lambda: skill_core.move_to_pose(
                    target_tcp, tcp_q0,
                    velocity_scale=0.10,
                    accel_scale=0.10,
                    orientation_tolerance=0.10,
                    position_tolerance=0.008,
                ),
                target_tcp, 0.012, True,
            )
            if (not ok1) or (not bool(hold_active_shared.value)):
                verification1["hold_active"] = bool(hold_active_shared.value)
                verification1.setdefault("reason", "SLOT_ENTRY_TRANSPORT_FAILED")
                return _v24_result(
                    name, False, contract, state_before, verification1, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            # One deterministic micro-correction is allowed; do not make the LLM
            # oscillate around the slot mouth.
            correction_used = False
            obj1, obj_q1 = skill_core.object_pose(obj)
            tcp1, tcp_q1 = skill_core.current_tcp_pose()
            xy1 = _v24_xy_distance(obj1, slot)
            if V271_SLOT_XY_TOL_M < xy1 <= 0.025:
                correction_used = True
                rel1 = [float(obj1[i]) - float(tcp1[i]) for i in range(3)]
                desired_obj2 = [float(slot[0]), float(slot[1]), float(plan["entry_object_center_z_m"])]
                target_tcp2 = [desired_obj2[i] - rel1[i] for i in range(3)]
                verification2, ok2, warn2 = _v24_run_motion(
                    f"v271_slot_entry_correction({obj},{slot_name})",
                    lambda: skill_core.move_to_pose(
                        target_tcp2, tcp_q1,
                        velocity_scale=0.06,
                        accel_scale=0.06,
                        orientation_tolerance=0.10,
                        position_tolerance=0.006,
                    ),
                    target_tcp2, 0.010, True,
                )
                warnings.extend(warn2)
                if (not ok2) or (not bool(hold_active_shared.value)):
                    verification2["hold_active"] = bool(hold_active_shared.value)
                    verification2["correction_used"] = True
                    verification2.setdefault("reason", "SLOT_ENTRY_CORRECTION_FAILED")
                    return _v24_result(
                        name, False, contract, state_before, verification2, warnings,
                        {"elapsed_s": round(time.perf_counter()-t0, 3)},
                        next_stage=None,
                        recommended_next_actions=["observe_scene"],
                    )

            obj2, obj_q2 = skill_core.object_pose(obj)
            tcp2, _ = skill_core.current_tcp_pose()
            xy_error = _v24_xy_distance(obj2, slot)
            axis2 = _v27_egg_major_axis_from_quat(obj_q2)["world_axis"]
            axis_error = _v271_axis_line_error_deg(axis2)
            rel_dist0 = _v24_xyz_distance(obj0, tcp0)
            rel_dist2 = _v24_xyz_distance(obj2, tcp2)
            rel_drift = abs(rel_dist2 - rel_dist0)
            ok = bool(
                xy_error <= V271_SLOT_XY_TOL_M
                and axis_error <= V271_AXIS_TOL_DEG
                and bool(hold_active_shared.value)
                and rel_drift <= V271_RELATIVE_DISTANCE_DRIFT_TOL_M
            )
            verification = {
                "slot_name": slot_name,
                "target_slot_xy_m": [float(slot[0]), float(slot[1])],
                "object_xyz_m": [round(float(v),5) for v in obj2],
                "object_xy_error_mm": round(xy_error*1000.0, 2),
                "xy_tolerance_mm": round(V271_SLOT_XY_TOL_M*1000.0, 1),
                "axis_error_deg": round(axis_error, 2),
                "axis_tolerance_deg": V271_AXIS_TOL_DEG,
                "correction_used": bool(correction_used),
                "object_tcp_distance_drift_mm": round(rel_drift*1000.0, 2),
                "hold_active": bool(hold_active_shared.value),
            }
            if not ok:
                verification["reason"] = "SLOT_ENTRY_ALIGNMENT_NOT_PRECISE_ENOUGH"
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "plan_placement"],
                )

            V271_PLACEMENT_RUNTIME["slot_entry"][obj] = {
                "slot_name": slot_name,
                "xy_error_mm": round(xy_error*1000.0,2),
                "axis_error_deg": round(axis_error,2),
            }
            skill_core.emit(
                f"[V2-7.1 SLOT ENTRY OK] {obj}->{slot_name} | xy_error={xy_error*1000.0:.1f}mm | "
                f"axis_error={axis_error:.1f}deg | correction={correction_used}"
            )
            return _v24_result(
                name, True, contract, state_before, verification, warnings,
                {"elapsed_s": round(time.perf_counter()-t0, 3)},
                "SLOT_ENTRY_ALIGNED", obj, f"egg_holder.{slot_name}",
                recommended_next_actions=["guarded_insert_and_release"],
            )

        if name == "guarded_insert_and_release":
            obj = str(args.get("object_name", state_before.get("active_object") or ""))
            slot_name = str(args.get("slot_name", ""))
            plan = V271_PLACEMENT_RUNTIME.get("plans", {}).get(obj)
            slot = _v271_slot(slot_name)
            if not plan or plan.get("slot_name") != slot_name or slot is None:
                return _v24_precondition_failure(name, args, "no matching placement plan", contract, state_before, recommended_next_actions=["plan_placement"])
            if state_before.get("stage") != "SLOT_ENTRY_ALIGNED" or state_before.get("active_object") != obj:
                return _v24_precondition_failure(
                    name, args, f"requires SLOT_ENTRY_ALIGNED; current={state_before}",
                    contract, state_before, recommended_next_actions=["move_to_slot_entry"],
                )
            if not bool(hold_active_shared.value):
                return _v24_precondition_failure(name, args, "hold_active=false before insertion", contract, state_before, recommended_next_actions=["observe_scene"])

            llm_require_arm_controller()
            safety0 = int(safety_trip_count.value)
            release_z = float(plan["pre_release_object_center_z_m"])
            records = []
            warnings = []
            insertion_ok = True
            abort_reason = None
            step_index = 0

            while True:
                obj_now, obj_q_now = skill_core.object_pose(obj)
                tcp_now, tcp_q_now = skill_core.current_tcp_pose()
                xy_error = _v24_xy_distance(obj_now, slot)
                axis_now = _v27_egg_major_axis_from_quat(obj_q_now)["world_axis"]
                axis_error = _v271_axis_line_error_deg(axis_now)
                remaining = float(obj_now[2]) - release_z

                if not bool(hold_active_shared.value):
                    insertion_ok = False
                    abort_reason = "GRASP_LOST_DURING_INSERTION"
                    break
                if int(safety_trip_count.value) != safety0:
                    insertion_ok = False
                    abort_reason = "SAFETY_TRIP_CHANGED_DURING_INSERTION"
                    break
                if xy_error > V271_SLOT_XY_ABORT_M:
                    insertion_ok = False
                    abort_reason = "LATERAL_SLOT_ERROR_DURING_INSERTION"
                    break
                if axis_error > V271_INSERT_AXIS_ABORT_DEG:
                    insertion_ok = False
                    abort_reason = "EGG_LONG_AXIS_DRIFT_DURING_INSERTION"
                    break
                if remaining <= 0.0025:
                    break

                dz = V271_INSERT_STEP_M if remaining > 0.020 else V271_INSERT_FINE_STEP_M
                target_obj_z = max(release_z, float(obj_now[2]) - dz)
                rel = [float(obj_now[i]) - float(tcp_now[i]) for i in range(3)]
                # Small deterministic XY correction keeps the egg centered while
                # the only intentional insertion direction is the slot/egg axis.
                target_obj = [float(slot[0]), float(slot[1]), target_obj_z]
                target_tcp = [target_obj[i] - rel[i] for i in range(3)]

                def _do_insert_step(target_tcp=target_tcp, tcp_q_now=tcp_q_now):
                    traj = skill_core._cartesian_approach_plan(target_tcp, tcp_q_now)
                    skill_core._execute_trajectory(traj)
                    return True

                verification_s, ok_s, warn_s = _v24_run_motion(
                    f"v271_insert_step_{step_index+1}({obj},{slot_name})",
                    _do_insert_step,
                    target_tcp, 0.008, True,
                )
                warnings.extend(warn_s)
                step_index += 1
                obj_after, obj_q_after = skill_core.object_pose(obj)
                xy_after = _v24_xy_distance(obj_after, slot)
                axis_after = _v271_axis_line_error_deg(_v27_egg_major_axis_from_quat(obj_q_after)["world_axis"])
                records.append({
                    "step": step_index,
                    "requested_object_center_z_mm": round(target_obj_z*1000.0, 1),
                    "actual_object_center_z_mm": round(float(obj_after[2])*1000.0, 1),
                    "xy_error_mm": round(xy_after*1000.0, 2),
                    "axis_error_deg": round(axis_after, 2),
                    "hold_active": bool(hold_active_shared.value),
                    "cartesian_ok": bool(ok_s),
                    "safety_trip_count": int(safety_trip_count.value),
                })
                skill_core.emit(
                    f"[V2-7.1 INSERT STEP] {obj}->{slot_name} | step={step_index} | "
                    f"z={float(obj_after[2])*1000.0:.1f}mm | xy={xy_after*1000.0:.1f}mm | "
                    f"axis={axis_after:.1f}deg | hold={bool(hold_active_shared.value)}"
                )
                if not ok_s:
                    insertion_ok = False
                    abort_reason = verification_s.get("reason", "INSERT_STEP_MOTION_FAILED")
                    break
                time.sleep(0.08)
                if step_index >= 12:
                    insertion_ok = False
                    abort_reason = "INSERT_STEP_LIMIT_EXCEEDED"
                    break

            if not insertion_ok:
                verification = {
                    "reason": abort_reason,
                    "slot_name": slot_name,
                    "insert_steps": records,
                    "release_performed": False,
                    "hold_active": bool(hold_active_shared.value),
                    "safety_trip_before": safety0,
                    "safety_trip_after": int(safety_trip_count.value),
                }
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            # Capture-depth reached.  Stop first, then release in two stages.
            time.sleep(0.25)
            obj_capture, obj_q_capture = skill_core.object_pose(obj)
            xy_capture = _v24_xy_distance(obj_capture, slot)
            axis_capture = _v271_axis_line_error_deg(_v27_egg_major_axis_from_quat(obj_q_capture)["world_axis"])
            if xy_capture > V271_SLOT_XY_ABORT_M or axis_capture > V271_INSERT_AXIS_ABORT_DEG:
                verification = {
                    "reason": "CAPTURE_DEPTH_ALIGNMENT_DEGRADED",
                    "object_xy_error_mm": round(xy_capture*1000.0,2),
                    "axis_error_deg": round(axis_capture,2),
                    "insert_steps": records,
                    "release_performed": False,
                }
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            opening0 = float(gripper_actual_mm.value)
            gripper_force_mode.value = False
            gripper_target_mm.value = min(100.0, opening0 + V271_RELEASE_STAGE_OPEN_MM)
            deadline = time.time() + 0.65
            while time.time() < deadline:
                if float(gripper_actual_mm.value) >= min(100.0, opening0 + 7.0):
                    break
                time.sleep(0.04)
            time.sleep(0.20)
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
            verification = {
                "slot_name": slot_name,
                "insert_steps": records,
                "pre_release_object_center_z_mm": round(float(obj_capture[2])*1000.0, 2),
                "planned_pre_release_center_z_mm": round(release_z*1000.0, 2),
                "capture_xy_error_mm": round(xy_capture*1000.0, 2),
                "capture_axis_error_deg": round(axis_capture, 2),
                "opening_before_release_mm": round(opening0, 2),
                "opening_after_release_mm": round(opening_after, 2),
                "hold_active_after_release": hold_after_release,
                "preliminary_location_after_release": preliminary_location,
                "location_after_retreat_settle": final_location,
                "post_release_retreat_ok": bool(retreat_ok),
                "post_release_retreat_error": retreat_error,
                "release_mechanically_ok": release_ok,
                "final_verification_required": True,
            }
            if not release_ok:
                verification["reason"] = "RELEASE_DID_NOT_COMPLETE"
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            V271_PLACEMENT_RUNTIME["released"][obj] = {
                "slot_name": slot_name,
                "preliminary_location": final_location,
            }
            skill_core.emit(
                f"[V2-7.1 RELEASE COMPLETE] {obj}->{slot_name} | capture_z={float(obj_capture[2])*1000.0:.1f}mm | "
                f"open={opening_after:.1f}mm | hold=false | location={final_location} | verify next"
            )
            return _v24_result(
                name, True, contract, state_before, verification, warnings,
                {"slot_name": slot_name, "elapsed_s": round(time.perf_counter()-t0,3)},
                V24_STAGE_RELEASED, obj, f"egg_holder.{slot_name}",
                recommended_next_actions=["verify_object_in_slot"],
            )

'''


AGENT_V271_GUIDE = r'''
V2-7.1 PLACEMENT-AWARE EGG POLICY:
- V2-7.1 keeps the V2-7 short-axis grasp logic.  The grasp orientation is NOT automatically the correct placement orientation.
- After a successful egg lift, egg placement into the slotted holder MUST use:
    plan_placement -> orient_for_slot -> move_to_slot_entry -> guarded_insert_and_release -> verify_object_in_slot
- Do NOT use legacy move_above_receptacle/place_into_receptacle/open_gripper for egg placement in V2-7.1.  Those remain only for backward/manual debug compatibility.
- plan_placement explicitly reasons about the held egg LONG axis and the slot entry axis.  It also computes a pre-release capture depth so the gripper does not have to descend all the way into the holder walls.
- orient_for_slot performs the large held-object rotation at a HIGH, open workspace pose, not at the slot mouth.  It keeps TCP XYZ approximately fixed during the reorientation and uses the current physical pose as the BioIK seed.  This encourages wrist-dominant J6/J7 motion but does not hard-code joint angles.
- move_to_slot_entry preserves the already-correct long-axis orientation and requires egg-center XY error <= 7 mm before insertion.  It may make one deterministic micro-correction; the LLM should not oscillate around the slot.
- guarded_insert_and_release performs the final descent in bounded 10 mm / 5 mm Cartesian segments along the egg/slot axis.  After every segment the backend checks hold, safety, XY error, and long-axis error.  At capture depth it stops, opens in two stages, retreats vertically, and leaves final success to verify_object_in_slot.
- If observe_scene detects a placement state but hold_active=false, the backend resets the stale placement state so a fresh pickup can begin.  Do not continue a placement after a dropped egg.
- If clamp_gripper reports BYSTANDER_EGG_DISTURBED, a non-target egg moved during approach.  Do not clamp/lift; re-observe and plan another grasp candidate.
- Egg breaking/tactile damage remains intentionally out of scope in V2-7.1.
'''


AGENT_V271_TOOLS = r'''
    {
        "type": "function",
        "name": "plan_placement",
        "description": "V2-7.1 PLACEMENT PLANNING — After lift, plan placement of the held egg into a selected empty egg-holder slot. Computes held egg LONG-axis alignment, safe high reorientation workspace, precise slot-entry height, and pre-release capture depth. Planning only; nothing moves.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "receptacle_name": {"type": "string", "enum": ["egg_holder"]},
                "slot_name": {"type": "string", "enum": ["slot_1", "slot_2", "slot_3", "slot_4", "slot_5", "slot_6", "slot_7", "slot_8"]}
            },
            "required": ["object_name", "receptacle_name", "slot_name"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "orient_for_slot",
        "description": "V2-7.1 HELD-OBJECT REORIENTATION — Move the held egg to a high safe workspace and rotate at approximately fixed TCP XYZ until the egg LONG axis aligns with the selected slot entry axis. BioIK is current-state seeded; wrist J6/J7 are preferred by minimum displacement but are not manually commanded.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "slot_name": {"type": "string", "enum": ["slot_1", "slot_2", "slot_3", "slot_4", "slot_5", "slot_6", "slot_7", "slot_8"]}
            },
            "required": ["object_name", "slot_name"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "move_to_slot_entry",
        "description": "V2-7.1 PRECISE SLOT ENTRY — With the egg already long-axis aligned, transport it above the selected slot while preserving orientation. Requires <=7 mm egg-center XY error before insertion and allows at most one deterministic micro-correction.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "slot_name": {"type": "string", "enum": ["slot_1", "slot_2", "slot_3", "slot_4", "slot_5", "slot_6", "slot_7", "slot_8"]}
            },
            "required": ["object_name", "slot_name"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "guarded_insert_and_release",
        "description": "V2-7.1 SLOW AXIAL INSERTION + RELEASE — From precise slot entry, descend the held egg along its LONG axis in bounded 10 mm then 5 mm Cartesian segments with hold/safety/XY/orientation checks. Stop at a pre-release capture depth above the final slot center, open the gripper in two stages, retreat vertically, then call verify_object_in_slot.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "slot_name": {"type": "string", "enum": ["slot_1", "slot_2", "slot_3", "slot_4", "slot_5", "slot_6", "slot_7", "slot_8"]}
            },
            "required": ["object_name", "slot_name"],
            "additionalProperties": False
        },
        "strict": True,
    },
'''


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Could not find patch anchor: {label}")
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if "NERO V2-7.1 — Placement-Aware Egg Handling" in text:
        raise RuntimeError("Backend already appears patched to V2-7.1")
    if "NERO V2-7 — Constraint-Aware Egg Grasp Planning" not in text:
        raise RuntimeError("Source backend does not appear to be the validated V2-7 runtime")

    # Module-level helpers before main().
    marker = "\ndef main():"
    if marker not in text:
        marker = "\ndef main("
    idx = text.find(marker)
    if idx < 0:
        raise RuntimeError("Could not find backend main() insertion point")
    text = text[:idx] + BACKEND_V271_HELPERS + "\n" + text[idx:]

    # New task-local runtime reset alongside V2-7 grasp runtime reset.
    reset_marker = "        _v27_reset_grasp_runtime(OBJECTS)\n"
    text = replace_once(
        text,
        reset_marker,
        reset_marker + "        _v271_reset_placement_runtime(OBJECTS)\n",
        "V2-7 reset grasp runtime",
    )

    # Capture other eggs at grasp-plan time so clamp can reject a candidate that
    # disturbed a bystander during approach.
    baseline_anchor = "            obj_xyz, obj_q = skill_core.object_pose(obj)\n            opening = float(gripper_actual_mm.value)\n"
    baseline_insert = (
        "            obj_xyz, obj_q = skill_core.object_pose(obj)\n"
        "            V271_BYSTANDER_BASELINE[obj] = {}\n"
        "            for _other in OBJECTS:\n"
        "                if str(_other) == obj or not str(_other).startswith('egg'):\n"
        "                    continue\n"
        "                try:\n"
        "                    _p_other, _q_other = skill_core.object_pose(_other)\n"
        "                    V271_BYSTANDER_BASELINE[obj][str(_other)] = [float(v) for v in _p_other]\n"
        "                except Exception:\n"
        "                    pass\n"
        "            opening = float(gripper_actual_mm.value)\n"
    )
    text = replace_once(text, baseline_anchor, baseline_insert, "V2-7 plan_grasp object-pose anchor")

    # Publish V2-7.1 placement state in every scene observation.
    obs_marker = '            scene["constraint_aware_grasp"] = _v27_public_runtime_snapshot()\n'
    text = replace_once(
        text,
        obs_marker,
        obs_marker + '            scene["placement_aware"] = _v271_public_runtime_snapshot()\n',
        "V2-7 observe_scene runtime snapshot",
    )

    # Contracts before legacy move_above contract.
    contract_marker = '            "move_above": {\n'
    text = replace_once(
        text,
        contract_marker,
        BACKEND_V271_CONTRACTS + contract_marker,
        "V2-7.1 skill contracts",
    )

    # Handlers after V2-7 candidate handlers and immediately before legacy move_above.
    handler_marker = '        if name == "move_above":\n'
    text = replace_once(
        text,
        handler_marker,
        BACKEND_V271_HANDLERS + handler_marker,
        "V2-7.1 tool handlers",
    )

    # Persistent log: reserve row 0 for a toolbar and move Text + scrollbar to row 1.
    text = replace_once(
        text,
        '    global_log_frame.rowconfigure(0, weight=1)\n',
        '    global_log_frame.rowconfigure(0, weight=0)\n    global_log_frame.rowconfigure(1, weight=1)\n',
        "global log rowconfigure",
    )
    text = replace_once(
        text,
        '    global_log.grid(row=0, column=0, sticky="nsew")\n',
        '    global_log.grid(row=1, column=0, sticky="nsew")\n',
        "global log text row",
    )
    text = replace_once(
        text,
        '    global_log_scroll.grid(row=0, column=1, sticky="ns")\n',
        '    global_log_scroll.grid(row=1, column=1, sticky="ns")\n',
        "global log scrollbar row",
    )
    log_anchor = (
        '    def append_gui_log(text):\n'
        '        global_log.insert("end", str(text).rstrip() + "\\n")\n'
        '        global_log.see("end")\n'
    )
    log_insert = log_anchor + r'''

    def copy_full_log_to_clipboard():
        try:
            payload = global_log.get("1.0", "end-1c")
            root.clipboard_clear()
            root.clipboard_append(payload)
            root.update_idletasks()
            append_gui_log(f"[GUI] Copied full runtime log to clipboard ({len(payload)} chars)")
        except Exception as exc:
            messagebox.showerror("NERO V2-7.1", f"Could not copy log:\n{exc}")

    global_log_toolbar = ttk.Frame(global_log_frame)
    global_log_toolbar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=4, pady=(3, 3))
    ttk.Button(
        global_log_toolbar,
        text="Copy Full Log",
        command=copy_full_log_to_clipboard,
    ).pack(side="left", padx=(0, 6))
    ttk.Label(
        global_log_toolbar,
        text="Copies the entire Unified Runtime Log to the system clipboard",
    ).pack(side="left")
'''
    text = replace_once(text, log_anchor, log_insert, "append_gui_log / Copy Full Log")

    # Import the renamed Agent module in the standalone V2-7.1 runtime.
    text = text.replace("from NERO_V2_7_Agent import", "from NERO_V2_7_1_Agent import")
    text = text.replace("NERO V2-7 |", "NERO V2-7.1 |")
    text = text.replace("NERO V2-7 Skill-Contract", "NERO V2-7.1 Skill-Contract")
    text = text.replace("NERO V2-7 unified runtime log", "NERO V2-7.1 unified runtime log")
    return text


def patch_agent(text: str) -> str:
    if "V2-7.1 PLACEMENT-AWARE EGG POLICY" in text:
        raise RuntimeError("Agent already appears patched to V2-7.1")
    if "V2-7 CONSTRAINT-AWARE EGG GRASP POLICY" not in text:
        raise RuntimeError("Source Agent does not appear to be the validated V2-7 Agent")

    text = text.replace('PROJECT_VERSION = "2.7.0"', 'PROJECT_VERSION = "2.7.1"')

    # Add new placement policy immediately before Operating policy.
    policy_marker = "\nOperating policy:\n"
    text = replace_once(
        text,
        policy_marker,
        "\n" + AGENT_V271_GUIDE + "\nOperating policy:\n",
        "Agent Operating policy",
    )

    # New tools are first-class tools; old placement tools remain in the file for
    # manual/backward compatibility but the system policy explicitly forbids them
    # for V2-7.1 egg placement.
    text = replace_once(
        text,
        "TOOLS = [\n",
        "TOOLS = [\n" + AGENT_V271_TOOLS,
        "Agent V2-7.1 tools",
    )

    # Strong override near the operating policy header.
    op = "Operating policy:\n"
    extra = (
        "Operating policy:\n"
        "-1. V2-7.1 EGG PLACEMENT OVERRIDE: after lift of an egg, use plan_placement -> orient_for_slot -> move_to_slot_entry -> guarded_insert_and_release -> verify_object_in_slot. Never use legacy move_above_receptacle/place_into_receptacle/open_gripper for V2-7.1 egg placement.\n"
    )
    text = replace_once(text, op, extra, "Agent V2-7.1 operating override")
    text = text.replace("NERO V2-7,", "NERO V2-7.1,")
    text = text.replace("NERO V2-7 ", "NERO V2-7.1 ")
    return text


def patch_launcher(text: str) -> str:
    text = text.replace("NERO_V2_7_Unified_Backend.py", "NERO_V2_7_1_Unified_Backend.py")
    text = text.replace("NERO_V2_7_Agent.py", "NERO_V2_7_1_Agent.py")
    text = text.replace("NERO V2-7", "NERO V2-7.1")
    text = text.replace("v2_7", "v2_7_1")
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "source_dir",
        nargs="?",
        default=".",
        help="Path to NERO_V2_7_Constraint_Aware_Grasp_Planning (default: current directory)",
    )
    ap.add_argument("--output-dir", default=None)
    args = ap.parse_args()

    src = Path(args.source_dir).expanduser().resolve()
    out = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else src.parent / "NERO_V2_7_1_Placement_Aware_Egg_Handling"
    )

    backend_in = src / "NERO_V2_7_Unified_Backend.py"
    agent_in = src / "NERO_V2_7_Agent.py"
    v69_in = src / "NERO_V69_ReachableZone_BioIK_Planner_Core.py"
    launcher_in = src / "start_nero_v2_7.sh"
    for p in (backend_in, agent_in, v69_in, launcher_in):
        if not p.is_file():
            raise SystemExit(f"Missing required V2-7 runtime file: {p}")

    v69_sha = sha256(v69_in)
    if v69_sha != EXPECTED_V69_SHA256:
        raise SystemExit(
            "Frozen V69 SHA mismatch. Refusing to build V2-7.1 from an unknown motion core.\n"
            f"expected={EXPECTED_V69_SHA256}\nactual={v69_sha}"
        )

    backend_text = backend_in.read_text(encoding="utf-8")
    agent_text = agent_in.read_text(encoding="utf-8")
    launcher_text = launcher_in.read_text(encoding="utf-8")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)

    backend_out = out / "NERO_V2_7_1_Unified_Backend.py"
    agent_out = out / "NERO_V2_7_1_Agent.py"
    launcher_out = out / "start_nero_v2_7_1.sh"

    backend_out.write_text(patch_backend(backend_text), encoding="utf-8")
    agent_out.write_text(patch_agent(agent_text), encoding="utf-8")
    launcher_out.write_text(patch_launcher(launcher_text), encoding="utf-8")
    launcher_out.chmod(launcher_out.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    # Keep only the new runtime entry-point files to reduce accidental launches
    # of the old version from inside the new folder.
    for old in (
        out / "NERO_V2_7_Unified_Backend.py",
        out / "NERO_V2_7_Agent.py",
        out / "start_nero_v2_7.sh",
    ):
        try:
            old.unlink()
        except FileNotFoundError:
            pass

    note = out / "V2_7_1_PLACEMENT_AWARE_EGG_HANDLING.md"
    note.write_text(
        "# NERO V2-7.1 — Placement-Aware Egg Handling\n\n"
        "V69 remains frozen. V2-7 grasp planning remains intact.\n\n"
        "New egg placement sequence:\n"
        "`plan_placement -> orient_for_slot -> move_to_slot_entry -> guarded_insert_and_release -> verify_object_in_slot`\n\n"
        "Key changes:\n"
        "- held egg long axis is aligned to slot entry axis at a safe high waypoint;\n"
        "- fixed-XYZ/current-seed reorientation encourages wrist-dominant motion without hard-coding J6/J7;\n"
        "- slot entry requires <=7 mm XY error and <=10 deg long-axis error;\n"
        "- insertion uses 10 mm / 5 mm Cartesian segments;\n"
        "- release occurs at a capture depth above the final slot center, then the gripper retreats;\n"
        "- non-target egg motion >3 mm blocks clamp;\n"
        "- stale placement state resets after an unexpected grasp loss;\n"
        "- GUI has a Copy Full Log button above the persistent log.\n\n"
        "Still out of scope: egg breaking, tactile damage, push/roll/nudge, generic skill synthesis.\n",
        encoding="utf-8",
    )

    # Static validation.
    py_compile.compile(str(backend_out), doraise=True)
    py_compile.compile(str(agent_out), doraise=True)
    if sha256(out / "NERO_V69_ReachableZone_BioIK_Planner_Core.py") != EXPECTED_V69_SHA256:
        raise SystemExit("V69 changed during packaging — abort")
    subprocess.run(["bash", "-n", str(launcher_out)], check=True)

    btxt = backend_out.read_text(encoding="utf-8")
    atxt = agent_out.read_text(encoding="utf-8")
    required_backend = [
        "V2-7.1 — Placement-Aware Egg Handling",
        'if name == "plan_placement":',
        'if name == "orient_for_slot":',
        'if name == "move_to_slot_entry":',
        'if name == "guarded_insert_and_release":',
        "BYSTANDER_EGG_DISTURBED",
        "Copy Full Log",
        "HOLD-LOSS STATE RECOVERY",
    ]
    required_agent = [
        "V2-7.1 PLACEMENT-AWARE EGG POLICY",
        '"name": "plan_placement"',
        '"name": "orient_for_slot"',
        '"name": "move_to_slot_entry"',
        '"name": "guarded_insert_and_release"',
    ]
    for marker in required_backend:
        if marker not in btxt:
            raise SystemExit(f"Static validation missing backend marker: {marker}")
    for marker in required_agent:
        if marker not in atxt:
            raise SystemExit(f"Static validation missing Agent marker: {marker}")

    print("[OK] NERO V2-7.1 package created:", out)
    print("[OK] Backend Python syntax")
    print("[OK] Agent Python syntax")
    print("[OK] Launcher bash syntax")
    print("[OK] Frozen V69 SHA256:", EXPECTED_V69_SHA256)
    print("[OK] Bystander Egg Guard")
    print("[OK] Long-axis placement planning")
    print("[OK] Safe-height held-object reorientation")
    print("[OK] Precise slot-entry alignment")
    print("[OK] Slow segmented axial insertion + staged release")
    print("[OK] Hold-loss state recovery")
    print("[OK] Copy Full Log GUI button")


if __name__ == "__main__":
    main()
