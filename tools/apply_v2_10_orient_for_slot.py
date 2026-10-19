#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import py_compile
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
BACKEND = "NERO_V2_7_1_Unified_Backend.py"
AGENT = "NERO_V2_7_1_Agent.py"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"


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
    if 'V272_VERSION = "2.7.2"' not in text:
        raise RuntimeError("Source backend is not the validated V2-7.2 runtime")
    if "V2-10 ORIENT TRANSFER" in text:
        raise RuntimeError("Backend already contains V2-10 orient patch")

    old = '''            # Step A: move the still-held egg to a high, open workspace while
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
'''

    new = '''            # V2-10 ORIENT TRANSFER — ONLY change versus V2-9/V2-7.2.
            # The old Step A asked MoveIt to solve one large diagonal transfer
            # (lift + ~155 mm lateral motion) while preserving a difficult held-
            # egg grasp orientation.  In the observed failure this returned
            # MoveIt FAILURE (99999) before wrist reorientation even started.
            #
            # V2-10 preserves the same final safe_xyz and the same Step-B wrist
            # reorientation.  It only decomposes transport into:
            #   A1) vertical clearance at current XY;
            #   A2) short high-Z XY segments (<=45 mm each) to safe_xyz.
            # No Agent, grasp, clamp, lift, wrist, slot-entry, insert, release,
            # MuJoCo, MoveIt-launch, controller, or V69 behavior is changed.
            warn_a = []
            transfer_records = []

            # A1 — gain clearance vertically first, preserving current grasp q0.
            vertical_xyz = [float(tcp0[0]), float(tcp0[1]), float(safe_xyz[2])]
            verification_up, ok_up, warn_up = _v24_run_motion(
                f"v210_orient_vertical_clearance({obj},{slot_name})",
                lambda: skill_core.move_to_pose(
                    vertical_xyz, q0,
                    velocity_scale=0.10,
                    accel_scale=0.10,
                    orientation_tolerance=0.12,
                    position_tolerance=0.010,
                ),
                vertical_xyz, 0.015, True,
            )
            warn_a.extend(warn_up)
            transfer_records.append({
                "phase": "vertical_clearance",
                "target_xyz_m": [round(float(v), 5) for v in vertical_xyz],
                "ok": bool(ok_up),
                "verification": verification_up,
            })
            if (not ok_up) or (not bool(hold_active_shared.value)):
                verification_a = dict(verification_up)
                verification_a["reason"] = verification_a.get("reason", "V210_VERTICAL_CLEARANCE_FAILED")
                verification_a["hold_active"] = bool(hold_active_shared.value)
                verification_a["v210_transfer_segments"] = transfer_records
                return _v24_result(
                    name, False, contract, state_before, verification_a, warn_a,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

            # Refresh actual high-pose orientation.  Keep this orientation fixed
            # during lateral transfer; Step B below remains the original V2-7.2
            # bounded wrist-reorientation implementation.
            tcp_high, q_high = skill_core.current_tcp_pose()
            dx = float(safe_xyz[0]) - float(tcp_high[0])
            dy = float(safe_xyz[1]) - float(tcp_high[1])
            horizontal_dist = math.sqrt(dx * dx + dy * dy)
            max_xy_step_m = 0.045
            n_xy = max(1, int(math.ceil(horizontal_dist / max_xy_step_m)))

            verification_a = dict(verification_up)
            ok_a = True
            for seg_idx in range(1, n_xy + 1):
                frac = float(seg_idx) / float(n_xy)
                seg_xyz = [
                    float(tcp_high[0]) + frac * dx,
                    float(tcp_high[1]) + frac * dy,
                    float(safe_xyz[2]),
                ]
                verification_seg, ok_seg, warn_seg = _v24_run_motion(
                    f"v210_orient_high_transfer_{seg_idx}of{n_xy}({obj},{slot_name})",
                    lambda seg_xyz=seg_xyz: skill_core.move_to_pose(
                        seg_xyz, q_high,
                        velocity_scale=0.10,
                        accel_scale=0.10,
                        orientation_tolerance=0.12,
                        position_tolerance=0.010,
                    ),
                    seg_xyz, 0.015, True,
                )
                warn_a.extend(warn_seg)
                transfer_records.append({
                    "phase": "high_xy_transfer",
                    "segment": seg_idx,
                    "segments_total": n_xy,
                    "target_xyz_m": [round(float(v), 5) for v in seg_xyz],
                    "ok": bool(ok_seg),
                    "verification": verification_seg,
                })
                verification_a = dict(verification_seg)
                ok_a = bool(ok_seg)
                if (not ok_seg) or (not bool(hold_active_shared.value)):
                    break

            verification_a["v210_transfer_segments"] = transfer_records
            verification_a["v210_horizontal_distance_mm"] = round(horizontal_dist * 1000.0, 2)
            verification_a["v210_xy_segments"] = int(n_xy)
            verification_a["hold_active"] = bool(hold_active_shared.value)

            if (not ok_a) or (not bool(hold_active_shared.value)):
                verification_a["reason"] = verification_a.get("reason", "V210_HIGH_XY_TRANSFER_FAILED")
                return _v24_result(
                    name, False, contract, state_before, verification_a, warn_a,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )
'''

    text = replace_once(text, old, new, "orient_for_slot Step A only")

    # Version marker only. Do not rename or mutate inherited V2-7.2 components.
    marker = 'V29_VERSION = "2.9"\n'
    if marker in text and 'V210_VERSION = "2.10"' not in text:
        text = text.replace(marker, marker + 'V210_VERSION = "2.10"\n', 1)
    return text


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runtime", type=Path)
    args = ap.parse_args()
    runtime = args.runtime.expanduser().resolve()

    backend = runtime / BACKEND
    agent = runtime / AGENT
    core = runtime / CORE
    launcher = runtime / "start_nero_v2_7_2.sh"
    for p in (backend, agent, core, launcher):
        if not p.is_file():
            raise FileNotFoundError(p)

    if sha256(core) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 hash mismatch; refusing V2-10 patch")

    agent_hash_before = sha256(agent)
    core_hash_before = sha256(core)
    launcher_hash_before = sha256(launcher)

    original = backend.read_text(encoding="utf-8")
    patched = patch_backend(original)
    backend.write_text(patched, encoding="utf-8")
    py_compile.compile(str(backend), doraise=True)

    if sha256(agent) != agent_hash_before:
        raise RuntimeError("V2-10 unexpectedly changed Agent")
    if sha256(core) != core_hash_before:
        raise RuntimeError("V2-10 unexpectedly changed frozen V69")
    if sha256(launcher) != launcher_hash_before:
        raise RuntimeError("V2-10 unexpectedly changed V2-7.2 runtime launcher")

    # Hard scope guards: exactly one V2-10 behavior marker and original Step-B
    # bounded wrist executor must still exist unchanged after the patch.
    check = backend.read_text(encoding="utf-8")
    if check.count("V2-10 ORIENT TRANSFER") != 1:
        raise RuntimeError("V2-10 orient marker count is not exactly one")
    for required in (
        "V210_VERTICAL_CLEARANCE_FAILED",
        "V210_HIGH_XY_TRANSFER_FAILED",
        "_v272_execute_wrist_bounded_pose(",
        'if name == "move_to_slot_entry":',
        'if name == "guarded_insert_and_release":',
    ):
        if required not in check:
            raise RuntimeError(f"Required inherited/runtime marker missing: {required}")

    print("[OK] V2-10 patched ONLY orient_for_slot Step A staged transfer", flush=True)
    print(f"[OK] backend={backend}", flush=True)
    print(f"[OK] Agent unchanged by V2-10: {agent_hash_before}", flush=True)
    print(f"[OK] V69 frozen: {core_hash_before}", flush=True)
    print(f"[OK] runtime launcher unchanged: {launcher_hash_before}", flush=True)


if __name__ == "__main__":
    main()
