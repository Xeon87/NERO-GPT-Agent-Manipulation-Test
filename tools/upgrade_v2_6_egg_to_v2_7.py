#!/usr/bin/env python3
"""Upgrade a local NERO V2-6 Egg Task Semantic Integration folder to V2-7.

This is intentionally a *small-step* upgrade:
- keeps NERO_V69_ReachableZone_BioIK_Planner_Core.py byte-for-byte unchanged;
- keeps the existing egg scene / egg-holder semantics / placement skills;
- adds constraint-aware egg grasp planning before the existing clamp/lift flow;
- does NOT model egg breaking, tactile damage, or non-prehensile egg reorientation.

Expected source folder:
  NERO_V2_6_Egg_Task_Semantic_Integration/
    NERO_V2_6_Unified_Backend.py
    NERO_V2_6_Agent.py
    NERO_V69_ReachableZone_BioIK_Planner_Core.py
    start_nero_v2_6.sh

The script creates a sibling folder:
  NERO_V2_7_Constraint_Aware_Grasp_Planning/

Run:
  python3 upgrade_v2_6_egg_to_v2_7.py /path/to/NERO_V2_6_Egg_Task_Semantic_Integration
"""

from __future__ import annotations

import argparse
import hashlib
import py_compile
import shutil
import stat
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"

BACKEND_V27_HELPERS = r'''

# ============================================================
# NERO V2-7 — Constraint-Aware Egg Grasp Planning
# ============================================================
# IMPORTANT:
#   V69 motion core remains frozen.  V2-7 uses already-existing V69 public/
#   internal planning hooks (move_to_pose, _plan_pose, _cartesian_approach_plan)
#   from the semantic wrapper only; NERO_V69_...py is not edited.
#
# Current scope:
#   - rigid eggs only; no break/damage model
#   - top-down two-finger grasp family
#   - egg major-axis orientation awareness
#   - analytical basket-rim clearance pre-check
#   - bounded alternate grasp candidates
#   - explicit failure when reorientation is required
#
# Deliberately deferred:
#   - push/roll/reorient-object skill
#   - tactile feedback / slip / pressure / damage
#   - learned grasp proposal network
# ============================================================

V27_BASKET_CENTER_XY_M = (-0.425, -0.155)
# Physical reed center radius is ~92 mm; reed radius ~4.2 mm.  Keep a few
# additional millimeters as a conservative projected-finger safety margin.
V27_BASKET_SAFE_INNER_RADIUS_M = 0.083

# Conservative XY footprint of the current V69 finger pad + studs.
# The V69 pad is ~60 mm long and ~29 mm wide; these values add a small margin.
V27_FINGER_HALF_LENGTH_M = 0.033
V27_FINGER_HALF_WIDTH_M = 0.017

V27_DEFAULT_GRASP_Z_OFFSET_M = 0.028
V27_PREGRASP_HEIGHT_M = 0.120
V27_OBJECT_STALE_POS_M = 0.008
V27_OBJECT_STALE_ANGLE_DEG = 15.0
V27_MAX_CANDIDATES_TO_PREFLIGHT = 8

V27_GRASP_RUNTIME = {
    "plans": {},               # object -> latest plan dict
    "tried": {},               # object -> set(candidate_id)
    "selected": {},            # object -> candidate_id
}


def _v27_reset_grasp_runtime(object_names=()):
    V27_GRASP_RUNTIME["plans"].clear()
    V27_GRASP_RUNTIME["selected"].clear()
    V27_GRASP_RUNTIME["tried"] = {str(n): set() for n in object_names}


def _v27_clamp(x, lo, hi):
    return max(float(lo), min(float(hi), float(x)))


def _v27_quat_normalize(q):
    x, y, z, w = [float(v) for v in q]
    n = (x*x + y*y + z*z + w*w) ** 0.5
    if n < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x/n, y/n, z/n, w/n]


def _v27_quat_multiply(a, b):
    ax, ay, az, aw = [float(v) for v in a]
    bx, by, bz, bw = [float(v) for v in b]
    return [
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ]


def _v27_top_down_quat(yaw_rad):
    # Exact same convention as V69 top_down_quat(yaw): q_yaw * Rx(pi).
    h = 0.5 * float(yaw_rad)
    q_yaw = [0.0, 0.0, math.sin(h), math.cos(h)]
    q_down = [1.0, 0.0, 0.0, 0.0]
    return _v27_quat_normalize(_v27_quat_multiply(q_yaw, q_down))


def _v27_axis_angle_pi(angle):
    """Normalize an undirected line angle to [-pi/2, pi/2)."""
    a = float(angle)
    while a >= math.pi / 2.0:
        a -= math.pi
    while a < -math.pi / 2.0:
        a += math.pi
    return a


def _v27_angle_diff_pi(a, b):
    return abs(_v27_axis_angle_pi(float(a) - float(b)))


def _v27_quat_rotation_angle_deg(a, b):
    qa = _v27_quat_normalize(a)
    qb = _v27_quat_normalize(b)
    d = abs(sum(float(qa[i]) * float(qb[i]) for i in range(4)))
    d = _v27_clamp(d, -1.0, 1.0)
    return math.degrees(2.0 * math.acos(d))


def _v27_egg_major_axis_from_quat(quat_xyzw):
    """Rotate local +Z (egg long axis) into world coordinates."""
    x, y, z, w = _v27_quat_normalize(quat_xyzw)
    # R(q) * [0,0,1]
    vx = 2.0 * (x*z + w*y)
    vy = 2.0 * (y*z - w*x)
    vz = 1.0 - 2.0 * (x*x + y*y)
    horizontal = math.hypot(vx, vy)
    if horizontal > 1e-9:
        yaw = _v27_axis_angle_pi(math.atan2(vy, vx))
    else:
        yaw = 0.0
    return {
        "world_axis": [vx, vy, vz],
        "horizontal_projection_strength": horizontal,
        "yaw_rad": yaw,
        "yaw_deg": math.degrees(yaw),
        "orientation_case": (
            "mostly_upright" if horizontal < 0.25
            else "tilted" if horizontal < 0.75
            else "mostly_horizontal"
        ),
    }


def _v27_gripper_footprint_clearance_m(grasp_xy, gripper_yaw_rad, opening_mm):
    """Projected finger-corner clearance to the circular wicker rim.

    Convention: gripper_yaw is the projected long-axis direction of each finger
    pad.  Finger closing direction is +90 deg from that axis.  Aligning the pad
    long axis with the egg major axis therefore pinches across the egg short axis.
    """
    gx, gy = [float(v) for v in grasp_xy]
    yaw = float(gripper_yaw_rad)
    long_dir = (math.cos(yaw), math.sin(yaw))
    close_dir = (-math.sin(yaw), math.cos(yaw))
    half_open = max(0.0, float(opening_mm) / 2000.0)

    points = []
    for finger_sign in (-1.0, +1.0):
        fcx = gx + finger_sign * half_open * close_dir[0]
        fcy = gy + finger_sign * half_open * close_dir[1]
        for ls in (-1.0, +1.0):
            for ws in (-1.0, +1.0):
                px = (
                    fcx
                    + ls * V27_FINGER_HALF_LENGTH_M * long_dir[0]
                    + ws * V27_FINGER_HALF_WIDTH_M * close_dir[0]
                )
                py = (
                    fcy
                    + ls * V27_FINGER_HALF_LENGTH_M * long_dir[1]
                    + ws * V27_FINGER_HALF_WIDTH_M * close_dir[1]
                )
                points.append((px, py))

    bx, by = V27_BASKET_CENTER_XY_M
    max_r = max(math.hypot(px - bx, py - by) for px, py in points)
    return V27_BASKET_SAFE_INNER_RADIUS_M - max_r


def _v27_make_grasp_candidates(object_name, obj_xyz, obj_quat, opening_mm, tried_ids=()):
    obj_xyz = [float(v) for v in obj_xyz]
    obj_quat = [float(v) for v in obj_quat]
    opening_mm = float(opening_mm)
    tried = {str(x) for x in tried_ids}

    axis = _v27_egg_major_axis_from_quat(obj_quat)
    ox, oy, oz = obj_xyz
    bx, by = V27_BASKET_CENTER_XY_M
    radial = math.atan2(oy - by, ox - bx)
    tangent = _v27_axis_angle_pi(radial + math.pi / 2.0)

    if axis["horizontal_projection_strength"] >= 0.25:
        base = float(axis["yaw_rad"])
        base_source = "egg_major_axis_projection"
    else:
        # Near-upright egg: its horizontal long-axis projection is ill-defined.
        # Prefer fingers tangent to the basket rim to reduce radial footprint.
        base = tangent
        base_source = "basket_tangent_for_near_upright_egg"

    yaw_defs = [
        ("major_aligned",          base,                         0),
        ("major_plus15",           base + math.radians(15.0),   1),
        ("major_minus15",          base - math.radians(15.0),   1),
        ("major_plus30",           base + math.radians(30.0),   2),
        ("major_minus30",          base - math.radians(30.0),   2),
        ("orthogonal_fallback",    base + math.pi / 2.0,        3),
        ("basket_tangent",         tangent,                     2),
    ]

    # Remove duplicate line orientations after pi-periodic normalization.
    unique = []
    for label, yaw, penalty_rank in yaw_defs:
        y = _v27_axis_angle_pi(yaw)
        if any(_v27_angle_diff_pi(y, old[1]) < math.radians(2.0) for old in unique):
            continue
        unique.append((label, y, penalty_rank))

    # Small inward grasp-center offset is allowed because a basket-wall egg may
    # otherwise leave insufficient room for the outer finger.  Keep it modest;
    # V2-7 does not intentionally push/reorient the egg.
    to_center_x = bx - ox
    to_center_y = by - oy
    n = math.hypot(to_center_x, to_center_y)
    if n > 1e-9:
        inward = (to_center_x / n, to_center_y / n)
    else:
        inward = (0.0, 0.0)

    obj_radial_r = math.hypot(ox - bx, oy - by)
    wall_near = (V27_BASKET_SAFE_INNER_RADIUS_M - obj_radial_r) < 0.030
    offset_defs = [("center", 0.0)]
    if wall_near:
        offset_defs += [("inward4", 0.004), ("inward6", 0.006)]
    else:
        offset_defs += [("inward4", 0.004)]

    candidates = []
    for label, yaw, yaw_rank in unique:
        for off_label, off in offset_defs:
            cid = f"{label}__{off_label}"
            gx = ox + inward[0] * off
            gy = oy + inward[1] * off
            clearance = _v27_gripper_footprint_clearance_m((gx, gy), yaw, opening_mm)
            geom_ok = clearance >= 0.0
            # Favor physically clear, major-axis-aligned, small-offset candidates.
            score = (clearance * 1000.0) - yaw_rank * 4.0 - off * 1000.0 * 0.60
            candidates.append({
                "candidate_id": cid,
                "object_name": str(object_name),
                "strategy": label,
                "base_orientation_source": base_source,
                "gripper_yaw_rad": float(yaw),
                "gripper_yaw_deg": round(math.degrees(yaw), 2),
                "closing_axis_yaw_deg": round(math.degrees(_v27_axis_angle_pi(yaw + math.pi/2.0)), 2),
                "grasp_center_offset_mm": round(off * 1000.0, 2),
                "grasp_xy_m": [round(gx, 6), round(gy, 6)],
                "grasp_xyz_m": [round(gx, 6), round(gy, 6), round(oz + V27_DEFAULT_GRASP_Z_OFFSET_M, 6)],
                "pregrasp_xyz_m": [round(gx, 6), round(gy, 6), round(oz + V27_DEFAULT_GRASP_Z_OFFSET_M + V27_PREGRASP_HEIGHT_M, 6)],
                "quat_xyzw": [round(float(v), 8) for v in _v27_top_down_quat(yaw)],
                "basket_projected_clearance_mm": round(clearance * 1000.0, 2),
                "geometry_feasible": bool(geom_ok),
                "already_tried": cid in tried,
                "score": round(score, 3),
            })

    candidates.sort(
        key=lambda c: (
            c["already_tried"],
            not c["geometry_feasible"],
            -float(c["score"]),
        )
    )
    return axis, candidates


def _v27_public_runtime_snapshot():
    out = {"selected": dict(V27_GRASP_RUNTIME.get("selected", {})), "tried": {}}
    for obj, ids in V27_GRASP_RUNTIME.get("tried", {}).items():
        out["tried"][obj] = sorted(str(x) for x in ids)
    return out
'''

BACKEND_V27_CONTRACTS = r'''
            "plan_grasp": {
                "purpose": f"Generate and preflight bounded constraint-aware grasp candidates for {obj} without moving the robot.",
                "preconditions": [
                    "fresh scene observation available",
                    "no object is currently held",
                    f"gripper aperture is already prepared for {obj}",
                ],
                "expected_effects": [
                    "estimate egg major-axis orientation from quaternion",
                    "generate bounded wrist-yaw / small inward-offset candidates",
                    "analytically reject candidates whose projected fingers intersect the basket rim",
                    "MoveIt-preflight remaining pregrasp poses without execution",
                ],
                "expected_non_effects": ["robot does not move", "egg does not move", "gripper does not move"],
                "success_criteria": ["at least one untried candidate is geometry-feasible and MoveIt-pregrasp-feasible"],
            },
            "move_to_pregrasp": {
                "purpose": f"Execute one previously planned V2-7 pregrasp candidate for {obj}.",
                "preconditions": ["plan_grasp succeeded", "candidate_id belongs to latest fresh plan", "object pose has not become stale"],
                "expected_effects": ["TCP reaches candidate pregrasp XYZ with candidate wrist yaw"],
                "expected_non_effects": [f"{obj} should remain stationary", "no grasp/contact should occur"],
                "success_criteria": ["TCP reaches pregrasp tolerance", "no new safety trip", "object remains approximately stationary"],
            },
            "approach_planned_grasp": {
                "purpose": f"Collision-checked straight descent from a V2-7 pregrasp candidate to its grasp pose for {obj}.",
                "preconditions": ["same candidate reached pregrasp", "candidate remains current", "gripper aperture remains prepared"],
                "expected_effects": ["TCP descends with fixed candidate wrist yaw to the planned grasp pose"],
                "expected_non_effects": [f"{obj} should not be intentionally pushed/reoriented in V2-7", "no clamp yet"],
                "success_criteria": ["TCP reaches grasp target", "pre-clamp object motion <= 4 mm", "no safety trip"],
            },
'''

BACKEND_V27_HANDLERS = r'''
        # ------------------------------------------------------------
        # NERO V2-7 Constraint-Aware Grasp Planning
        # ------------------------------------------------------------
        if name == "plan_grasp":
            stage = state_before.get("stage")
            if stage not in (V24_STAGE_OBSERVED, V24_STAGE_HOVER, V24_STAGE_VERIFIED, V24_STAGE_RELEASED):
                return _v24_precondition_failure(
                    name, args,
                    f"plan_grasp not allowed from state={stage}; observe/recover first",
                    contract, state_before,
                    recommended_next_actions=["observe_scene"],
                )
            if bool(hold_active_shared.value):
                return _v24_precondition_failure(
                    name, args, "hold_active=true; cannot plan a new pickup while holding an object",
                    contract, state_before,
                    recommended_next_actions=["resolve_or_release_held_object"],
                )
            obj = str(args.get("object_name", ""))
            if obj not in OBJECTS:
                return _v24_precondition_failure(
                    name, args, f"unknown object {obj}; valid={list(OBJECTS)}",
                    contract, state_before, recommended_next_actions=["observe_scene"],
                )
            if not obj.startswith("egg"):
                return _v24_precondition_failure(
                    name, args, f"V2-7 plan_grasp currently supports egg objects only; got {obj}",
                    contract, state_before, recommended_next_actions=["use_legacy_skill_or_stop"],
                )
            aperture = _v24_aperture_check(obj)
            if not aperture["robust_aperture_ok"]:
                return _v24_precondition_failure(
                    name, args,
                    f"prepare gripper aperture before grasp planning: {aperture}",
                    contract, state_before,
                    physical_verification={"gripper_aperture": aperture},
                    recommended_next_actions=["set_gripper_opening", "observe_scene", "plan_grasp"],
                )

            obj_xyz, obj_q = skill_core.object_pose(obj)
            opening = float(gripper_actual_mm.value)
            tried = V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set())
            axis_info, candidates = _v27_make_grasp_candidates(obj, obj_xyz, obj_q, opening, tried)

            preflighted = []
            untried_count = 0
            for cand in candidates:
                c = dict(cand)
                c["moveit_pregrasp_ok"] = False
                c["moveit_pregrasp_error"] = None
                if c["already_tried"]:
                    preflighted.append(c)
                    continue
                untried_count += 1
                if not c["geometry_feasible"]:
                    c["moveit_pregrasp_error"] = "BASKET_RIM_PROJECTED_CLEARANCE_INSUFFICIENT"
                    preflighted.append(c)
                    continue
                if untried_count > V27_MAX_CANDIDATES_TO_PREFLIGHT:
                    c["moveit_pregrasp_error"] = "PREFLIGHT_BUDGET_NOT_REACHED"
                    preflighted.append(c)
                    continue
                try:
                    # Plan only; do NOT execute.  V69 core is unchanged.
                    skill_core._plan_pose(
                        c["pregrasp_xyz_m"], c["quat_xyzw"],
                        velocity_scale=0.18, accel_scale=0.18,
                        orientation_tolerance=0.12,
                        position_tolerance=0.010,
                        planning_attempts=3,
                        planning_time=2.5,
                    )
                    c["moveit_pregrasp_ok"] = True
                except Exception as exc:
                    c["moveit_pregrasp_error"] = str(exc)
                preflighted.append(c)

            feasible = [
                c for c in preflighted
                if (not c["already_tried"])
                and c["geometry_feasible"]
                and c["moveit_pregrasp_ok"]
            ]
            feasible.sort(key=lambda c: -float(c["score"]))
            recommended = feasible[0] if feasible else None

            plan = {
                "object_name": obj,
                "object_xyz_at_plan_m": [round(float(v), 6) for v in obj_xyz],
                "object_quat_at_plan_xyzw": [round(float(v), 8) for v in obj_q],
                "egg_major_axis": axis_info,
                "gripper_opening_mm": round(opening, 2),
                "basket_center_xy_m": list(V27_BASKET_CENTER_XY_M),
                "basket_safe_inner_radius_mm": round(V27_BASKET_SAFE_INNER_RADIUS_M * 1000.0, 1),
                "candidate_count": len(preflighted),
                "tried_candidate_ids": sorted(str(x) for x in tried),
                "candidates": preflighted,
                "recommended_candidate_id": recommended["candidate_id"] if recommended else None,
                "reorientation_required_if_none": True,
            }
            V27_GRASP_RUNTIME["plans"][obj] = plan
            if recommended is not None:
                V27_GRASP_RUNTIME["selected"][obj] = recommended["candidate_id"]
                _v24_set_state(active_object=obj)
                verification = {
                    "planning_only": True,
                    "physical_motion_expected": False,
                    "recommended_candidate_id": recommended["candidate_id"],
                    "recommended_gripper_yaw_deg": recommended["gripper_yaw_deg"],
                    "recommended_basket_clearance_mm": recommended["basket_projected_clearance_mm"],
                }
                skill_core.emit(
                    f"[V2-7 GRASP PLAN] {obj} | major_axis_yaw={axis_info['yaw_deg']:.1f}deg | "
                    f"orientation={axis_info['orientation_case']} | selected={recommended['candidate_id']} | "
                    f"gripper_yaw={recommended['gripper_yaw_deg']:.1f}deg | "
                    f"basket_clearance={recommended['basket_projected_clearance_mm']:.1f}mm"
                )
                return _v24_result(
                    name, True, contract, state_before, verification, [],
                    {"grasp_plan": plan, "elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["move_to_pregrasp"],
                )

            verification = {
                "planning_only": True,
                "physical_motion_expected": False,
                "reason": "ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE",
                "reorientation_required": True,
                "reorientation_skill_available": False,
            }
            skill_core.emit(
                f"[V2-7 GRASP INFEASIBLE] {obj} | no untried candidate passed basket-clearance + MoveIt preflight | "
                "REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7"
            )
            return _v24_result(
                name, False, contract, state_before, verification,
                ["No bounded V2-7 grasp candidate is currently executable. Do not loop. Object reorientation is intentionally deferred."],
                {"grasp_plan": plan, "elapsed_s": round(time.perf_counter()-t0, 3)},
                next_stage=None,
                recommended_next_actions=["stop_this_object_and_report_REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7"],
            )

        if name == "move_to_pregrasp":
            obj = str(args.get("object_name", ""))
            cid = str(args.get("candidate_id", ""))
            if bool(hold_active_shared.value):
                return _v24_precondition_failure(
                    name, args, "hold_active=true; cannot move to a new pregrasp",
                    contract, state_before, recommended_next_actions=["resolve_or_release_held_object"],
                )
            plan = V27_GRASP_RUNTIME.get("plans", {}).get(obj)
            if not plan:
                return _v24_precondition_failure(
                    name, args, f"no current V2-7 grasp plan for {obj}",
                    contract, state_before, recommended_next_actions=["observe_scene", "plan_grasp"],
                )
            cand_map = {c["candidate_id"]: c for c in plan.get("candidates", [])}
            cand = cand_map.get(cid)
            if cand is None:
                return _v24_precondition_failure(
                    name, args, f"candidate_id={cid} not found in latest plan",
                    contract, state_before, recommended_next_actions=["plan_grasp"],
                )
            if cand.get("already_tried") or cid in V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()):
                return _v24_precondition_failure(
                    name, args, f"candidate {cid} was already tried; do not repeat it",
                    contract, state_before, recommended_next_actions=["plan_grasp"],
                )
            if not (cand.get("geometry_feasible") and cand.get("moveit_pregrasp_ok")):
                return _v24_precondition_failure(
                    name, args, f"candidate {cid} is not preflight-feasible: {cand}",
                    contract, state_before, recommended_next_actions=["plan_grasp"],
                )

            xyz_now, q_now = skill_core.object_pose(obj)
            pos_stale = _v24_xyz_distance(xyz_now, plan["object_xyz_at_plan_m"])
            ang_stale = _v27_quat_rotation_angle_deg(q_now, plan["object_quat_at_plan_xyzw"])
            if pos_stale > V27_OBJECT_STALE_POS_M or ang_stale > V27_OBJECT_STALE_ANGLE_DEG:
                V27_GRASP_RUNTIME["plans"].pop(obj, None)
                return _v24_precondition_failure(
                    name, args,
                    f"grasp plan stale: object moved {pos_stale*1000.0:.1f}mm / rotated {ang_stale:.1f}deg",
                    contract, state_before,
                    physical_verification={"object_motion_since_plan_mm": round(pos_stale*1000.0,2), "object_rotation_since_plan_deg": round(ang_stale,2)},
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            llm_require_arm_controller()
            obj_before = list(xyz_now)
            target = [float(v) for v in cand["pregrasp_xyz_m"]]
            q = [float(v) for v in cand["quat_xyzw"]]
            verification, cart_ok, warnings = _v24_run_motion(
                f"move_to_pregrasp({obj},{cid})",
                lambda: skill_core.move_to_pose(
                    target, q,
                    velocity_scale=0.18,
                    accel_scale=0.18,
                    orientation_tolerance=0.12,
                    position_tolerance=0.010,
                ),
                target, 0.012, True,
            )
            obj_after, _ = skill_core.object_pose(obj)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            verification.update({
                "candidate_id": cid,
                "candidate_gripper_yaw_deg": cand["gripper_yaw_deg"],
                "basket_projected_clearance_mm": cand["basket_projected_clearance_mm"],
                "object_motion_mm": round(object_move*1000.0, 2),
                "object_motion_expected": False,
            })
            ok = bool(cart_ok and object_move <= 0.005)
            if not ok:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                verification.setdefault("reason", "PREGRASP_CANDIDATE_EXECUTION_FAILED")
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            V27_GRASP_RUNTIME["selected"][obj] = cid
            skill_core.active_grasp_quat = list(q)
            return _v24_result(
                name, True, contract, state_before, verification, warnings,
                {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                V24_STAGE_HOVER, obj,
                recommended_next_actions=["approach_planned_grasp"],
            )

        if name == "approach_planned_grasp":
            obj = str(args.get("object_name", ""))
            cid = str(args.get("candidate_id", ""))
            if state_before.get("stage") != V24_STAGE_HOVER or state_before.get("active_object") != obj:
                return _v24_precondition_failure(
                    name, args, f"requires HOVER_REACHED for same object; current={state_before}",
                    contract, state_before, recommended_next_actions=["plan_grasp", "move_to_pregrasp"],
                )
            plan = V27_GRASP_RUNTIME.get("plans", {}).get(obj)
            if not plan:
                return _v24_precondition_failure(
                    name, args, f"no current V2-7 grasp plan for {obj}",
                    contract, state_before, recommended_next_actions=["observe_scene", "plan_grasp"],
                )
            cand_map = {c["candidate_id"]: c for c in plan.get("candidates", [])}
            cand = cand_map.get(cid)
            if cand is None or V27_GRASP_RUNTIME.get("selected", {}).get(obj) != cid:
                return _v24_precondition_failure(
                    name, args, f"candidate {cid} is not the currently reached/selected pregrasp",
                    contract, state_before, recommended_next_actions=["plan_grasp", "move_to_pregrasp"],
                )
            aperture = _v24_aperture_check(obj)
            if not aperture["robust_aperture_ok"]:
                return _v24_precondition_failure(
                    name, args, f"aperture changed outside robust range: {aperture}",
                    contract, state_before,
                    physical_verification={"gripper_aperture": aperture},
                    recommended_next_actions=["set_gripper_opening", "observe_scene", "plan_grasp"],
                )

            xyz_now, q_now = skill_core.object_pose(obj)
            pos_stale = _v24_xyz_distance(xyz_now, plan["object_xyz_at_plan_m"])
            ang_stale = _v27_quat_rotation_angle_deg(q_now, plan["object_quat_at_plan_xyzw"])
            if pos_stale > V27_OBJECT_STALE_POS_M or ang_stale > V27_OBJECT_STALE_ANGLE_DEG:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                return _v24_precondition_failure(
                    name, args,
                    f"object pose became stale before descent: moved {pos_stale*1000.0:.1f}mm / rotated {ang_stale:.1f}deg",
                    contract, state_before,
                    physical_verification={"object_motion_since_plan_mm": round(pos_stale*1000.0,2), "object_rotation_since_plan_deg": round(ang_stale,2)},
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            llm_require_arm_controller()
            obj_before = list(xyz_now)
            opening_before = float(gripper_actual_mm.value)
            target = [float(v) for v in cand["grasp_xyz_m"]]
            q = [float(v) for v in cand["quat_xyzw"]]

            def _do_v27_descent():
                traj = skill_core._cartesian_approach_plan(target, q)
                skill_core._execute_trajectory(traj)
                return True

            verification, cart_ok, warnings = _v24_run_motion(
                f"approach_planned_grasp({obj},{cid})",
                _do_v27_descent,
                target, 0.010, True,
            )
            obj_after, _ = skill_core.object_pose(obj)
            opening_after = float(gripper_actual_mm.value)
            object_move = _v24_xyz_distance(obj_after, obj_before)
            object_move_mm = object_move * 1000.0
            verification.update({
                "candidate_id": cid,
                "candidate_gripper_yaw_deg": cand["gripper_yaw_deg"],
                "basket_projected_clearance_mm": cand["basket_projected_clearance_mm"],
                "object_motion_mm": round(object_move_mm, 2),
                "object_motion_required": False,
                "gripper_opening_before_approach_mm": round(opening_before, 2),
                "gripper_opening_after_approach_mm": round(opening_after, 2),
            })

            motion_error = str(verification.get("legacy_joint_verify_error", ""))
            if (not cart_ok) and "CARTESIAN APPROACH BLOCKED" in motion_error:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                verification["reason"] = "GRASP_CANDIDATE_APPROACH_BLOCKED"
                verification["candidate_marked_tried"] = True
                warnings.append("This candidate is exhausted. Re-observe and request a different constraint-aware candidate; do not repeat it.")
                skill_core.emit(f"[V2-7 CANDIDATE BLOCKED] {obj} {cid} | candidate exhausted; replan another yaw/offset")
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            safety_stop = bool(
                verification.get("reason") == "SAFETY_TRIP_CHANGED"
                or "PHYSICAL_SAFETY_STOP" in str(verification.get("legacy_joint_verify_error", ""))
                or "PHYSICAL_SAFETY_STOP" in str(verification.get("motion_exception", ""))
            )
            if safety_stop:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                used = int(V242_SAFETY_RETRY_USED.get(obj, 0))
                retry_allowed = bool(used < 1 and not bool(hold_active_shared.value))
                ctx = _v242_set_safety_context(
                    active=True,
                    tool="approach_planned_grasp",
                    object_name=obj,
                    trip_count=int(safety_trip_count.value),
                    safety_state_code=int(safety_state.value),
                    likely_cause=(
                        "SELECTED_GRASP_CANDIDATE_CONTACTED_BASKET_OR_OBJECT; "
                        "candidate is exhausted and a different yaw/offset must be planned"
                    ),
                    confidence="high",
                    evidence=[
                        f"candidate={cid}",
                        f"predicted_basket_clearance={cand['basket_projected_clearance_mm']}mm",
                        f"object_motion_before_clamp={object_move_mm:.1f}mm",
                    ],
                    suggested_strategy="retreat_and_retry_once" if retry_allowed else "stop",
                    suggested_retry_opening_mm=float(aperture["recommended_target_opening_mm"]),
                    retry_allowed=retry_allowed,
                    retry_used=used,
                    retry_limit=1,
                )
                verification["reason"] = "PREGRASP_SAFETY_STOP_REQUIRES_ALTERNATE_CANDIDATE"
                verification["safety_reflection_context"] = ctx
                _v24_set_state(V24_STAGE_SAFETY, active_object=obj, target_bin=None, last_tool=name, last_ok=False)
                return _v24_result(
                    name, False, contract, state_before, verification,
                    warnings + ["Do not retry the same candidate after recovery."],
                    {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "recover_from_safety"] if retry_allowed else ["stop_and_explain_failure"],
                )

            if object_move_mm > 4.0:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                verification["reason"] = "PREGRASP_CONTACT_OR_OBJECT_DISPLACEMENT"
                verification["candidate_marked_tried"] = True
                warnings.append(
                    f"Object moved {object_move_mm:.1f}mm before clamp. This candidate is exhausted; choose another candidate instead of repeating."
                )
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            if not cart_ok:
                V27_GRASP_RUNTIME.setdefault("tried", {}).setdefault(obj, set()).add(cid)
                verification.setdefault("reason", "GRASP_CANDIDATE_APPROACH_FAILED")
                return _v24_result(
                    name, False, contract, state_before, verification, warnings,
                    {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene", "plan_grasp"],
                )

            skill_core.active_grasp_quat = list(q)
            return _v24_result(
                name, True, contract, state_before, verification, warnings,
                {"object_name": obj, "candidate_id": cid, "elapsed_s": round(time.perf_counter()-t0,3)},
                V24_STAGE_GRASP_POSE, obj,
                recommended_next_actions=["clamp_gripper"],
            )

'''

AGENT_V27_GUIDE = r'''
V2-7 CONSTRAINT-AWARE EGG GRASP POLICY:
- For egg pickup from the basket, V2-7 adds a mandatory grasp-planning step.  Do not treat an egg as a point target.
- After observe_scene and aperture preparation, call plan_grasp(object_name).
- plan_grasp is observation/planning only.  It estimates the egg major-axis orientation from the current quaternion, models the two-finger projected footprint relative to the circular wicker rim, generates a bounded set of wrist-yaw / small inward-offset candidates, and MoveIt-preflights their pregrasp poses without executing motion.
- The preferred family aligns the finger-pad long axis with the egg's projected major axis so the closing direction pinches across the egg short axis.  Near-upright eggs use a basket-tangent fallback because horizontal egg yaw is underdetermined.
- plan_grasp returns candidate IDs and a recommended candidate.  Use move_to_pregrasp(object_name, candidate_id), then approach_planned_grasp(object_name, candidate_id).
- A candidate is never repeated after it is marked tried/blocked/contacting.  Re-observe and call plan_grasp so the backend chooses among the remaining bounded candidates.
- For V2-7 egg pickup, do NOT bypass the candidate layer by repeatedly calling legacy move_above/approach.  Those remain available for manual/debug compatibility only.
- If plan_grasp returns ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE / REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7, STOP that object.  V2-7 intentionally does not yet push, roll, or nudge the egg into a new orientation.  Never loop.
- Egg breaking / damage is intentionally out of scope in V2-7.  Treat eggs as rigid ellipsoids for this version.
'''

AGENT_V27_TOOLS = r'''
    {
        "type": "function",
        "name": "plan_grasp",
        "description": "V2-7 CONSTRAINT-AWARE PLANNING — For an egg in the basket, generate bounded grasp candidates from egg orientation, gripper aperture and basket-rim clearance, then MoveIt-preflight candidate pregrasp poses WITHOUT moving the robot. Returns candidate IDs plus recommended_candidate_id. Mandatory before V2-7 egg pickup.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]}
            },
            "required": ["object_name"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "move_to_pregrasp",
        "description": "V2-7 CANDIDATE EXECUTION — Move the TCP to a specific candidate pregrasp pose returned by the latest plan_grasp. Applies candidate wrist yaw and small center offset while preserving the V69 safety/motion stack. Object should remain stationary.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "candidate_id": {"type": "string", "minLength": 3}
            },
            "required": ["object_name", "candidate_id"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "approach_planned_grasp",
        "description": "V2-7 CANDIDATE DESCENT — From the reached candidate pregrasp, execute a collision-checked straight descent to that candidate's grasp pose with fixed wrist yaw. If blocked/contacting, the candidate is exhausted and must not be repeated; re-observe and plan another candidate.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["egg_1", "egg_2", "egg_3", "egg_4"]},
                "candidate_id": {"type": "string", "minLength": 3}
            },
            "required": ["object_name", "candidate_id"],
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
    n = text.count(old)
    if n < 1:
        raise RuntimeError(f"Patch marker not found for {label!r}: {old[:120]!r}")
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if "V2-7 — Constraint-Aware Egg Grasp Planning" in text:
        raise RuntimeError("Backend already appears patched to V2-7")

    # Version/name cleanup first; do not touch the V69 filename/hash.
    text = text.replace("NERO_V2_6_Agent", "NERO_V2_7_Agent")
    text = text.replace("NERO V2-6", "NERO V2-7")
    text = text.replace("V2-6", "V2-7")

    # Module-level geometry / candidate helpers.
    marker = "\ndef main():"
    if marker not in text:
        marker = "\ndef main("
    idx = text.find(marker)
    if idx < 0:
        raise RuntimeError("Could not find backend main() insertion point")
    text = text[:idx] + BACKEND_V27_HELPERS + "\n" + text[idx:]

    # Clear V2-7 candidate/tried state at the same explicit episode boundary as
    # the existing V2-6/V2-7 safety/planning retry state.
    reset_marker = "        _v242_clear_safety_context()\n"
    text = replace_once(
        text,
        reset_marker,
        "        _v27_reset_grasp_runtime(OBJECTS)\n" + reset_marker,
        "episode reset grasp runtime",
    )

    # Expose tried/selected candidate memory in observations for debugging.
    obs_marker = '            scene["v2_skill_state"] = _v24_state_snapshot()\n'
    text = replace_once(
        text,
        obs_marker,
        obs_marker + '            scene["constraint_aware_grasp"] = _v27_public_runtime_snapshot()\n',
        "observe_scene V2-7 runtime snapshot",
    )

    # Add contracts before legacy move_above contract.
    contract_marker = '            "move_above": {\n'
    text = replace_once(
        text,
        contract_marker,
        BACKEND_V27_CONTRACTS + contract_marker,
        "V2-7 skill contracts",
    )

    # Add tool handlers immediately before legacy move_above.  Existing
    # move_above/approach remain for manual/debug compatibility.
    handler_marker = '        if name == "move_above":\n'
    text = replace_once(
        text,
        handler_marker,
        BACKEND_V27_HANDLERS + handler_marker,
        "V2-7 tool handlers",
    )

    # Safety recovery should return to fresh candidate planning rather than the
    # legacy one-target approach after a V2-7 candidate contact.
    old_next = '["set_gripper_opening", "observe_scene", "move_above", "approach"]'
    if old_next in text:
        text = text.replace(old_next, '["set_gripper_opening", "observe_scene", "plan_grasp"]')

    return text


def patch_agent(text: str) -> str:
    if "V2-7 CONSTRAINT-AWARE EGG GRASP POLICY" in text:
        raise RuntimeError("Agent already appears patched to V2-7")

    text = text.replace('PROJECT_VERSION = "2.6.0"', 'PROJECT_VERSION = "2.7.0"')
    text = text.replace("NERO V2-6", "NERO V2-7")
    text = text.replace("V2-6", "V2-7")

    # Insert the policy before the existing state-machine/contract section.
    if "\nState machine:\n" in text:
        text = text.replace("\nState machine:\n", "\n" + AGENT_V27_GUIDE + "\nState machine:\n", 1)
    elif "\nContract rules:\n" in text:
        text = text.replace("\nContract rules:\n", "\n" + AGENT_V27_GUIDE + "\nContract rules:\n", 1)
    else:
        raise RuntimeError("Could not find Agent guide insertion point")

    # Add tools at the front of the tool list. observe_scene is still required
    # as the first *called* action by policy.
    text = replace_once(
        text,
        "TOOLS = [\n",
        "TOOLS = [\n" + AGENT_V27_TOOLS,
        "Agent V2-7 tools",
    )

    # Add a strong execution rule near the general operating policy without
    # trying to rewrite the existing semantic-integration instructions.
    policy_marker = "Operating policy:\n"
    extra = (
        "Operating policy:\n"
        "0. V2-7 EGG PICKUP OVERRIDE: for egg_1..egg_4 in the basket, after the mandatory observe_scene and aperture preparation, use plan_grasp -> move_to_pregrasp -> approach_planned_grasp before clamp_gripper. Do not repeatedly use legacy move_above/approach for egg pickup.\n"
    )
    text = replace_once(text, policy_marker, extra, "Agent V2-7 operating policy")

    return text


def patch_launcher(text: str) -> str:
    text = text.replace("NERO_V2_6_Unified_Backend.py", "NERO_V2_7_Unified_Backend.py")
    text = text.replace("NERO_V2_6_Agent.py", "NERO_V2_7_Agent.py")
    text = text.replace("NERO V2-6", "NERO V2-7")
    text = text.replace("V2-6", "V2-7")
    text = text.replace("v2_6", "v2_7")
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "source_dir",
        nargs="?",
        default=".",
        help="Path to NERO_V2_6_Egg_Task_Semantic_Integration folder (default: current directory)",
    )
    ap.add_argument(
        "--output-dir",
        default=None,
        help="Optional output folder. Default: sibling NERO_V2_7_Constraint_Aware_Grasp_Planning",
    )
    args = ap.parse_args()

    src = Path(args.source_dir).expanduser().resolve()
    required = [
        "NERO_V2_6_Unified_Backend.py",
        "NERO_V2_6_Agent.py",
        "NERO_V69_ReachableZone_BioIK_Planner_Core.py",
        "start_nero_v2_6.sh",
    ]
    missing = [name for name in required if not (src / name).is_file()]
    if missing:
        raise SystemExit(
            "Source folder is not the expected V2-6 Egg Semantic package. Missing:\n  - "
            + "\n  - ".join(missing)
            + f"\nSource checked: {src}"
        )

    v69_src = src / "NERO_V69_ReachableZone_BioIK_Planner_Core.py"
    v69_hash = sha256(v69_src)
    if v69_hash != EXPECTED_V69_SHA256:
        raise SystemExit(
            "V69 hash mismatch; refusing to patch because this upgrade promises a frozen V69 core.\n"
            f"Expected: {EXPECTED_V69_SHA256}\nActual:   {v69_hash}"
        )

    out = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir
        else src.parent / "NERO_V2_7_Constraint_Aware_Grasp_Planning"
    )
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True, exist_ok=True)

    # Preserve every non-Python/non-launcher package file (docs, examples,
    # config, etc.) so V2-7 is a direct derivative of the semantic package.
    for item in src.iterdir():
        if item.name in required:
            continue
        dst = out / item.name
        if item.is_dir():
            shutil.copytree(item, dst)
        else:
            shutil.copy2(item, dst)

    backend_text = (src / "NERO_V2_6_Unified_Backend.py").read_text(encoding="utf-8")
    agent_text = (src / "NERO_V2_6_Agent.py").read_text(encoding="utf-8")
    launcher_text = (src / "start_nero_v2_6.sh").read_text(encoding="utf-8")

    backend_out = out / "NERO_V2_7_Unified_Backend.py"
    agent_out = out / "NERO_V2_7_Agent.py"
    launcher_out = out / "start_nero_v2_7.sh"
    core_out = out / "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

    backend_out.write_text(patch_backend(backend_text), encoding="utf-8")
    agent_out.write_text(patch_agent(agent_text), encoding="utf-8")
    launcher_out.write_text(patch_launcher(launcher_text), encoding="utf-8")
    shutil.copy2(v69_src, core_out)
    launcher_out.chmod(launcher_out.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    notes = out / "V2_7_CONSTRAINT_AWARE_GRASP.md"
    notes.write_text(
        """# NERO V2-7 — Constraint-Aware Grasp Planning\n\n"
        "V69 core remains byte-for-byte frozen.\n\n"
        "New egg-pick flow:\n\n"
        "`observe_scene -> set_gripper_opening -> plan_grasp -> move_to_pregrasp -> approach_planned_grasp -> clamp_gripper -> lift -> existing receptacle placement flow`\n\n"
        "V2-7 adds egg-orientation-aware wrist yaw, a conservative projected two-finger basket-rim clearance model, bounded alternate candidates, MoveIt preflight, candidate exhaustion, and explicit `REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7` instead of infinite retry.\n\n"
        "Egg breaking, tactile feedback, and push/roll reorientation are intentionally deferred.\n",
        encoding="utf-8",
    )

    # Syntax checks.  These do not require ROS/MuJoCo imports to be installed.
    py_compile.compile(str(backend_out), doraise=True)
    py_compile.compile(str(agent_out), doraise=True)

    copied_hash = sha256(core_out)
    if copied_hash != EXPECTED_V69_SHA256:
        raise SystemExit("Internal error: copied V69 hash changed")

    print("[OK] NERO V2-7 package created:", out)
    print("[OK] Backend syntax:", backend_out.name)
    print("[OK] Agent syntax:", agent_out.name)
    print("[OK] Frozen V69 SHA256:", copied_hash)
    print("[OK] Launcher:", launcher_out.name)
    print("\nFirst test command:")
    print("  Pick up egg_1 and place it into the first empty egg-holder slot.")
    print("\nRun:")
    print(f"  cd {out}")
    print("  ./start_nero_v2_7.sh")


if __name__ == "__main__":
    main()
