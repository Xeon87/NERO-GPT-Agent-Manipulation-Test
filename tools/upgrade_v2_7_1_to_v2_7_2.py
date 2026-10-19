#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import math
import py_compile
import shutil
import stat
import subprocess
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
BACKEND = "NERO_V2_6_Unified_Backend.py"
AGENT = "NERO_V2_6_Agent.py"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

HELPERS = r'''
# ============================================================
# NERO V2-7.2 — Hold Continuity + Bounded Wrist Reorientation
# ============================================================
V272_VERSION = "2.7.2"
V272_WRIST_STEP_DEG = 10.0
V272_WRIST_PLAN_ATTEMPTS = 4
V272_JOINT_STEP_LIMIT_DEG = {
    "joint1": 4.0, "joint2": 4.0, "joint3": 4.0,
    "joint4": 8.0, "joint5": 10.0,
    "joint6": 25.0, "joint7": 25.0,
}
V272_JOINT_SCORE_WEIGHT = {
    "joint1": 8.0, "joint2": 8.0, "joint3": 8.0,
    "joint4": 4.0, "joint5": 3.0,
    "joint6": 1.0, "joint7": 1.0,
}


def _v272_wrap_delta(v):
    x = float(v)
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def _v272_plan_joint_deltas_deg(skill_core, traj):
    if traj is None:
        return {}
    jt = traj.joint_trajectory
    if not jt.points or not jt.joint_names:
        return {}
    js = skill_core._current_joint_state_copy(timeout_s=1.5)
    cur = {str(n): float(p) for n, p in zip(js.name, js.position)}
    final = {str(n): float(p) for n, p in zip(jt.joint_names, jt.points[-1].positions)}
    out = {}
    for name in [f"joint{i}" for i in range(1, 8)]:
        if name in cur and name in final:
            out[name] = math.degrees(abs(_v272_wrap_delta(final[name] - cur[name])))
    return out


def _v272_wrist_score(d):
    return sum(float(V272_JOINT_SCORE_WEIGHT.get(k, 1.0)) * float(v) for k, v in d.items())


def _v272_wrist_plan_ok(d):
    if not d:
        return False
    for name, lim in V272_JOINT_STEP_LIMIT_DEG.items():
        if float(d.get(name, 0.0)) > float(lim):
            return False
    proximal = sum(float(d.get(f"joint{i}", 0.0)) for i in range(1, 6))
    return proximal <= 22.0


def _v272_execute_wrist_bounded_pose(skill_core, xyz, quat_xyzw,
                                     velocity_scale=0.035,
                                     accel_scale=0.035,
                                     orientation_tolerance=0.07,
                                     position_tolerance=0.006):
    """Plan first; reject large shoulder/elbow branch changes before execution.

    V69 is untouched.  We use its existing _plan_pose/_execute_trajectory,
    inspect the final joint solution, and execute only a bounded branch.
    """
    accepted = []
    rejected = []
    for attempt in range(1, int(V272_WRIST_PLAN_ATTEMPTS) + 1):
        traj = skill_core._plan_pose(
            xyz, quat_xyzw,
            velocity_scale=velocity_scale,
            accel_scale=accel_scale,
            orientation_tolerance=orientation_tolerance,
            position_tolerance=position_tolerance,
            planning_attempts=12,
            planning_time=5.0,
        )
        d = _v272_plan_joint_deltas_deg(skill_core, traj)
        rec = {
            "attempt": attempt,
            "joint_delta_deg": {k: round(float(v), 2) for k, v in d.items()},
            "score": round(_v272_wrist_score(d), 2) if d else None,
            "bounded": bool(_v272_wrist_plan_ok(d)),
        }
        if rec["bounded"]:
            accepted.append((_v272_wrist_score(d), traj, rec))
        else:
            rejected.append(rec)
    if not accepted:
        skill_core.emit(
            "[V2-7.2 WRIST PREFLIGHT REJECT] no bounded branch | " +
            json.dumps(rejected, ensure_ascii=False)
        )
        raise RuntimeError(
            "WRIST_BOUNDED_PLAN_REJECTED: all IK/MoveIt branches require excessive joint motion"
        )
    accepted.sort(key=lambda x: x[0])
    _score, traj, chosen = accepted[0]
    skill_core.emit(
        "[V2-7.2 WRIST PREFLIGHT OK] joint_delta_deg=" +
        json.dumps(chosen["joint_delta_deg"], ensure_ascii=False) +
        f" | weighted_score={chosen['score']}"
    )
    return skill_core._execute_trajectory(traj)


def _v272_digit_segments(digit):
    yL, yR = -0.0048, 0.0048
    zB, zM, zT = -0.0065, 0.0, 0.0065
    if digit == 1:
        return [((0.0015, zT), (0.0015, zB)), ((-0.0015, zT-0.0015), (0.0015, zT))]
    if digit == 2:
        return [((yL, zT), (yR, zT)), ((yR, zT), (yL, zB)), ((yL, zB), (yR, zB))]
    if digit == 3:
        return [((yL, zT), (yR, zT)), ((yL+0.001, zM), (yR, zM)),
                ((yL, zB), (yR, zB)), ((yR, zT), (yR, zM)), ((yR, zM), (yR, zB))]
    if digit == 4:
        return [((yL, zT), (yL, zM)), ((yL, zM), (yR, zM)), ((yR, zT), (yR, zB))]
    return []


def _v272_add_egg_number_labels_to_mjcf(xml_path):
    """Attach non-colliding 3D numerals 1/2/3/4 to the actual four egg bodies."""
    path = Path(xml_path).resolve()
    tree = ET.parse(path)
    root = tree.getroot()
    found = []
    for digit in range(1, 5):
        body_name = f"obj_egg{digit}"
        body = root.find(f".//body[@name='{body_name}']")
        if body is None:
            continue
        found.append(body_name)
        for old in list(body.findall("geom")):
            if str(old.get("name", "")).startswith(f"v272_egg{digit}_label_"):
                body.remove(old)
        idx = 0
        for side in (-1.0, 1.0):
            x = side * 0.0235
            for (y0, z0), (y1, z1) in _v272_digit_segments(digit):
                idx += 1
                ET.SubElement(body, "geom", {
                    "name": f"v272_egg{digit}_label_{idx:02d}",
                    "type": "capsule",
                    "fromto": f"{x:.5f} {y0:.5f} {z0:.5f} {x:.5f} {y1:.5f} {z1:.5f}",
                    "size": "0.00085",
                    "rgba": "0.035 0.035 0.035 1",
                    "contype": "0", "conaffinity": "0", "density": "0", "group": "1",
                })
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(f"[V2-7.2 EGG IDENTITY] labelled bodies={found} count={len(found)} expected=4", flush=True)
    if len(found) != 4:
        raise RuntimeError(f"Expected exactly 4 named egg bodies, found {len(found)}: {found}")
    return str(path)
'''

CONTINUITY = r'''        def deactivate_all_holds():
            # V2-7.2 HOLD-STATE CONTINUITY.  Snapshot the CURRENT displayed
            # world pose before eq_active goes to zero, then seed that pose back
            # into the free joint.  This blocks old-pose snapback/teleport.
            _snap = []
            for _bid, _eqid in hold_eq_by_body.items():
                try:
                    if int(data.eq_active[_eqid]) == 0:
                        continue
                    _jid = int(model.body_jntadr[int(_bid)])
                    if _jid < 0 or int(model.jnt_type[_jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
                        continue
                    _qadr = int(model.jnt_qposadr[_jid])
                    _dadr = int(model.jnt_dofadr[_jid])
                    _pos = np.array(data.xpos[int(_bid)], dtype=float).copy()
                    _quat = np.array(data.xquat[int(_bid)], dtype=float).copy()
                    _snap.append((int(_bid), _qadr, _dadr, _pos, _quat))
                except Exception:
                    pass
            for eqid in hold_eq_by_body.values():
                data.eq_active[eqid] = 0
            if _snap:
                for _bid, _qadr, _dadr, _pos, _quat in _snap:
                    data.qpos[_qadr:_qadr+3] = _pos
                    data.qpos[_qadr+3:_qadr+7] = _quat
                    data.qvel[_dadr:_dadr+6] = 0.0
                mujoco.mj_forward(model, data)
                _names = [hold_name_by_body.get(int(x[0]), f"body#{int(x[0])}") for x in _snap]
                print("[V2-7.2 HOLD CONTINUITY] released at CURRENT pose: " + str(_names), flush=True)
'''

EARLY_RECOVERY = r'''        state_before = _v24_state_snapshot()
        # V2-7.2: recover stale carrying state BEFORE an older observe handler can return.
        if (
            name == "observe_scene"
            and not bool(hold_active_shared.value)
            and str(state_before.get("stage") or "") in (
                "LIFTED", "RECEPTACLE_HOVER_REACHED", "SLOT_PLACE_REACHED",
                "PLACEMENT_ORIENTED", "SLOT_ENTRY_ALIGNED",
            )
        ):
            _lost = str(state_before.get("active_object") or "")
            skill_core.emit(
                f"[V2-7.2 GRASP LOST RECOVERY] stage={state_before.get('stage')} "
                f"active_object={_lost or None} hold_active=false -> clear stale state"
            )
            try:
                if _lost:
                    for _bucket in ("plans", "oriented", "slot_entry", "released"):
                        V271_PLACEMENT_RUNTIME.get(_bucket, {}).pop(_lost, None)
            except Exception:
                pass
            _v24_reset_skill_state("V2-7.2 physical grasp loss", clear_adaptation=False)
            state_before = _v24_state_snapshot()
'''


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {n}; refusing partial patch")
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if 'V272_VERSION = "2.7.2"' in text:
        raise RuntimeError("Backend already contains V2-7.2 marker")

    text = replace_once(
        text, "\nV271_SLOT_CENTERS_M = {",
        "\n" + HELPERS + "\n\nV271_SLOT_CENTERS_M = {",
        "insert V2-7.2 helpers",
    )

    old = """        def deactivate_all_holds():\n            for eqid in hold_eq_by_body.values():\n                data.eq_active[eqid] = 0\n"""
    text = replace_once(text, old, CONTINUITY, "hold continuity")

    old = """        visual_model_file = make_grid_world_xml(MODEL_FILE)\n        visual_model_file = add_official_gripper_to_xml(visual_model_file)\n"""
    new = """        visual_model_file = make_grid_world_xml(MODEL_FILE)\n        visual_model_file = _v272_add_egg_number_labels_to_mjcf(visual_model_file)\n        visual_model_file = add_official_gripper_to_xml(visual_model_file)\n"""
    text = replace_once(text, old, new, "egg labels")

    old = """        state_before = _v24_state_snapshot()\n        contract = _v24_contract(name, args)\n"""
    text = replace_once(text, old, EARLY_RECOVERY + "        contract = _v24_contract(name, args)\n", "early grasp-loss recovery")

    old = "            n_steps = 1 if rotation_deg <= 35.0 else (2 if rotation_deg <= 70.0 else 3)\n"
    new = """            n_steps = max(1, int(math.ceil(float(rotation_deg) / float(V272_WRIST_STEP_DEG))))\n"""
    text = replace_once(text, old, new, "orientation step count")

    old = """                    lambda q_step=q_step: skill_core.move_to_pose(\n                        safe_xyz, q_step,\n                        velocity_scale=0.07,\n                        accel_scale=0.07,\n                        orientation_tolerance=0.10,\n                        position_tolerance=0.008,\n                    ),\n"""
    new = """                    lambda q_step=q_step: _v272_execute_wrist_bounded_pose(\n                        skill_core, safe_xyz, q_step,\n                        velocity_scale=0.035,\n                        accel_scale=0.035,\n                        orientation_tolerance=0.07,\n                        position_tolerance=0.006,\n                    ),\n"""
    text = replace_once(text, old, new, "wrist-bounded executor")

    text = text.replace(
        "rotate at approximately fixed TCP XYZ in 1-3 bounded orientation steps",
        "rotate at approximately fixed TCP XYZ in ~10-degree bounded orientation steps",
    )
    text = text.replace(
        "use current-pose-seeded BioIK; wrist-dominant motion is preferred but J6/J7 are not hard-coded",
        "use current-pose-seeded BioIK; reject excessive J1-J5 branches before execution; J6/J7 carry most reorientation when feasible",
    )
    text = text.replace(
        '"joint_solution": "BioIK current-state seed; J6/J7 preferred by minimum displacement, not hard-coded"',
        '"joint_solution": "BioIK + MoveIt preflight; excessive proximal-arm branches rejected; frozen V69 unchanged"',
    )
    text = text.replace("NERO V2-7.1", "NERO V2-7.2")
    text = text.replace("[V2-7.1 ", "[V2-7.2 ")
    return text


def patch_agent(text: str) -> str:
    addition = '''\nV2-7.2 placement safety additions:\n- Egg identities are physical labels 1/2/3/4 attached to the actual egg bodies.\n- Held-object reorientation is slow and segmented (~10 deg/step).\n- Each orientation target is planned first; branches with excessive J1-J5 movement are rejected before execution.\n- If WRIST_BOUNDED_PLAN_REJECTED occurs, do not loop the same motion. Observe and report feasibility failure.\n- If hold_active becomes false during transport/placement, backend clears stale carrying state before the next observation.\n- Safety stop or hold loss is never placement success.\n'''
    anchor = "\nOperating policy:"
    if "V2-7.2 placement safety additions:" not in text:
        if anchor not in text:
            raise RuntimeError("Agent patch anchor missing: Operating policy")
        text = text.replace(anchor, addition + anchor, 1)
    text = text.replace("NERO V2-7.1", "NERO V2-7.2")
    text = text.replace("1-3 bounded orientation steps", "~10-degree bounded orientation steps")
    return text


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()
    src = args.source_dir.expanduser().resolve()
    out = args.output_dir.expanduser().resolve()

    for name in (BACKEND, AGENT, CORE):
        if not (src / name).is_file():
            raise FileNotFoundError(f"Missing validated V2-7.1 runtime file: {src/name}")
    if sha256(src / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 SHA mismatch; refusing to patch")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)

    bp = out / BACKEND
    apath = out / AGENT
    bp.write_text(patch_backend(bp.read_text(encoding="utf-8")), encoding="utf-8")
    apath.write_text(patch_agent(apath.read_text(encoding="utf-8")), encoding="utf-8")

    old_launcher = out / "start_nero_v2_7_1.sh"
    new_launcher = out / "start_nero_v2_7_2.sh"
    if old_launcher.is_file():
        s = old_launcher.read_text(encoding="utf-8")
        s = s.replace("V2-7.1", "V2-7.2")
        s = s.replace("NERO_V2_7_1_Placement_Aware_Egg_Handling", out.name)
        new_launcher.write_text(s, encoding="utf-8")
    else:
        new_launcher.write_text('#!/usr/bin/env bash\nset -euo pipefail\ncd "$(dirname "$0")"\nexec python3 NERO_V2_6_Unified_Backend.py\n', encoding="utf-8")
    new_launcher.chmod(new_launcher.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    py_compile.compile(str(bp), doraise=True)
    py_compile.compile(str(apath), doraise=True)
    subprocess.run(["bash", "-n", str(new_launcher)], check=True)
    if sha256(out / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during copy/patch")

    final = bp.read_text(encoding="utf-8")
    markers = [
        'V272_VERSION = "2.7.2"',
        "V2-7.2 HOLD CONTINUITY",
        "V2-7.2 WRIST PREFLIGHT",
        "_v272_add_egg_number_labels_to_mjcf",
        "V2-7.2 GRASP LOST RECOVERY",
    ]
    missing = [m for m in markers if m not in final]
    if missing:
        raise RuntimeError(f"V2-7.2 marker validation failed: {missing}")

    (out / "V2_7_2_NOTES.md").write_text(
        "# NERO V2-7.2\n\n"
        "- Frozen V69 unchanged.\n"
        "- Four physical eggs carry visible 1/2/3/4 labels on both sides.\n"
        "- HOLD release re-seeds current world pose to prevent old-pose snapback.\n"
        "- Stale LIFTED state is cleared after physical grasp loss.\n"
        "- Held-egg orientation uses ~10 degree steps with joint-branch preflight.\n"
        "- Egg breaking/tactile/push-roll remain out of scope.\n",
        encoding="utf-8",
    )

    print("[OK] NERO V2-7.2 runtime created:", out)
    print("[OK] Backend syntax")
    print("[OK] Agent syntax")
    print("[OK] Launcher syntax")
    print("[OK] Frozen V69 SHA256:", EXPECTED_V69_SHA256)
    print("[OK] HOLD continuity + egg identity + bounded wrist patches installed")
    print("[RUN]", new_launcher)


if __name__ == "__main__":
    main()
