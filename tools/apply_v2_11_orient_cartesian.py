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
LAUNCHER = "start_nero_v2_7_2.sh"

START_MARKER = "            # V2-10 ORIENT TRANSFER"
END_MARKER = "            # Step B: at essentially fixed XYZ, rotate the complete held-object"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def patch_backend(text: str) -> str:
    if "V2-11 ORIENT CARTESIAN TRANSFER" in text:
        raise RuntimeError("Backend already contains V2-11 orient patch")
    if START_MARKER not in text:
        raise RuntimeError("Source backend is not the tested V2-10 runtime: V2-10 orient marker missing")
    if text.count(START_MARKER) != 1:
        raise RuntimeError(f"Expected exactly one V2-10 orient block, found {text.count(START_MARKER)}")
    if text.count(END_MARKER) != 1:
        raise RuntimeError(f"Expected exactly one inherited Step-B anchor, found {text.count(END_MARKER)}")

    i0 = text.index(START_MARKER)
    i1 = text.index(END_MARKER, i0)
    if i1 <= i0:
        raise RuntimeError("Invalid orient_for_slot marker ordering")

    new_block = '''            # V2-11 ORIENT CARTESIAN TRANSFER — ONLY behavior change versus V2-10.
            # Keep the held egg on the CURRENT local IK branch during transport.
            # Build ONE GetCartesianPath trajectory containing:
            #   A1) short vertical-clearance waypoints at current XY;
            #   A2) short high-Z XY waypoints to the original safe_xyz.
            # The current ACTUAL TCP quaternion q0 is fixed over the whole path.
            # Step B below (V2-7.2 bounded wrist reorientation) is untouched.
            transport_start = [float(v) for v in tcp0]
            transport_q = [float(v) for v in q0]
            transport_waypoints = []

            # A1: vertical clearance, <=25 mm between explicit waypoints.
            dz_clear = float(safe_xyz[2]) - float(transport_start[2])
            n_up = 0
            if dz_clear > 0.001:
                n_up = max(1, int(math.ceil(dz_clear / 0.025)))
                for k in range(1, n_up + 1):
                    f = float(k) / float(n_up)
                    transport_waypoints.append([
                        float(transport_start[0]),
                        float(transport_start[1]),
                        float(transport_start[2]) + dz_clear * f,
                    ])

            # A2: high-Z XY translation, <=25 mm between explicit waypoints.
            high_start = [
                float(transport_start[0]),
                float(transport_start[1]),
                float(safe_xyz[2]),
            ]
            dx = float(safe_xyz[0]) - float(high_start[0])
            dy = float(safe_xyz[1]) - float(high_start[1])
            horizontal_dist = math.sqrt(dx * dx + dy * dy)
            n_xy = 0
            if horizontal_dist > 0.001:
                n_xy = max(1, int(math.ceil(horizontal_dist / 0.025)))
                for k in range(1, n_xy + 1):
                    f = float(k) / float(n_xy)
                    transport_waypoints.append([
                        float(high_start[0]) + dx * f,
                        float(high_start[1]) + dy * f,
                        float(safe_xyz[2]),
                    ])

            if not transport_waypoints:
                transport_waypoints = [[float(v) for v in safe_xyz]]

            def _v211_execute_cartesian_transport():
                skill_core.emit(
                    f"[V2-11 CARTESIAN TRANSPORT] {obj}->{slot_name} | "
                    f"vertical={max(0.0, dz_clear)*1000.0:.1f}mm/{n_up}wp | "
                    f"horizontal={horizontal_dist*1000.0:.1f}mm/{n_xy}wp | "
                    f"total_waypoints={len(transport_waypoints)} | fixed actual TCP quaternion"
                )
                traj = skill_core._cartesian_waypoint_plan(
                    transport_waypoints,
                    transport_q,
                    label=f"V2-11 held-egg safe transport {obj}->{slot_name}",
                    max_step=0.003,
                    max_speed=0.035,
                )
                return skill_core._execute_trajectory(traj)

            verification_a, ok_a, warn_a = _v24_run_motion(
                f"v211_orient_cartesian_transport({obj},{slot_name})",
                _v211_execute_cartesian_transport,
                safe_xyz, 0.015, True,
            )
            verification_a["v211_transport_mode"] = "single_continuous_cartesian"
            verification_a["v211_fixed_actual_tcp_quaternion"] = [
                round(float(v), 7) for v in transport_q
            ]
            verification_a["v211_vertical_clearance_mm"] = round(max(0.0, dz_clear) * 1000.0, 2)
            verification_a["v211_vertical_waypoints"] = int(n_up)
            verification_a["v211_horizontal_distance_mm"] = round(horizontal_dist * 1000.0, 2)
            verification_a["v211_horizontal_waypoints"] = int(n_xy)
            verification_a["v211_total_waypoints"] = len(transport_waypoints)
            verification_a["hold_active"] = bool(hold_active_shared.value)

            if (not ok_a) or (not bool(hold_active_shared.value)):
                verification_a["reason"] = verification_a.get(
                    "reason", "V211_CARTESIAN_TRANSPORT_FAILED"
                )
                return _v24_result(
                    name, False, contract, state_before, verification_a, warn_a,
                    {"elapsed_s": round(time.perf_counter()-t0, 3)},
                    next_stage=None,
                    recommended_next_actions=["observe_scene"],
                )

'''

    patched = text[:i0] + new_block + text[i1:]

    forbidden = (
        "v210_orient_vertical_clearance",
        "v210_orient_high_transfer_",
        "V210_VERTICAL_CLEARANCE_FAILED",
        "V210_HIGH_XY_TRANSFER_FAILED",
    )
    for marker in forbidden:
        if marker in patched:
            raise RuntimeError(f"Old V2-10 transport marker unexpectedly remains: {marker}")

    required = (
        "V2-11 ORIENT CARTESIAN TRANSFER",
        "_cartesian_waypoint_plan(",
        "_v272_execute_wrist_bounded_pose(",
        'if name == "move_to_slot_entry":',
        'if name == "guarded_insert_and_release":',
    )
    for marker in required:
        if marker not in patched:
            raise RuntimeError(f"Required inherited/runtime marker missing: {marker}")

    return patched


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runtime", type=Path)
    args = ap.parse_args()
    runtime = args.runtime.expanduser().resolve()

    backend = runtime / BACKEND
    agent = runtime / AGENT
    core = runtime / CORE
    launcher = runtime / LAUNCHER
    for path in (backend, agent, core, launcher):
        if not path.is_file():
            raise FileNotFoundError(path)

    if sha256(core) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 hash mismatch; refusing V2-11 patch")

    agent_before = sha256(agent)
    core_before = sha256(core)
    launcher_before = sha256(launcher)

    original = backend.read_text(encoding="utf-8")
    patched = patch_backend(original)
    backend.write_text(patched, encoding="utf-8")
    py_compile.compile(str(backend), doraise=True)

    if sha256(agent) != agent_before:
        raise RuntimeError("V2-11 unexpectedly changed Agent")
    if sha256(core) != core_before:
        raise RuntimeError("V2-11 unexpectedly changed frozen V69")
    if sha256(launcher) != launcher_before:
        raise RuntimeError("V2-11 unexpectedly changed V2-7.2 runtime launcher")

    print("[OK] V2-11 patched ONLY orient_for_slot transport")
    print("[OK] Transport now uses one continuous Cartesian trajectory")
    print("[OK] Fixed quaternion = current actual TCP quaternion at orient_for_slot start")
    print("[OK] Agent unchanged:", agent_before)
    print("[OK] V69 frozen:", core_before)
    print("[OK] runtime launcher unchanged:", launcher_before)


if __name__ == "__main__":
    main()
