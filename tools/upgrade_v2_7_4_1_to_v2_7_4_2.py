#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import py_compile
import shutil
import stat
import subprocess
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"

SRC_BACKEND = "NERO_V2_7_4_1_Unified_Backend.py"
SRC_AGENT = "NERO_V2_7_4_1_Agent.py"
SRC_LAUNCHER = "start_nero_v2_7_4_1.sh"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

OUT_BACKEND = "NERO_V2_7_4_2_Unified_Backend.py"
OUT_AGENT = "NERO_V2_7_4_2_Agent.py"
OUT_LAUNCHER = "start_nero_v2_7_4_2.sh"
OUT_DIR_NAME = "NERO_V2_7_4_2_MuJoCo_Viewer_Handshake"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {n}")
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if 'V2741_VERSION = "2.7.4.1"' not in text:
        raise RuntimeError("Source backend is not validated V2-7.4.1")

    # Spawned MuJoCo process must import the viewer submodule explicitly.
    text = replace_once(
        text,
        "import mujoco\n",
        "import mujoco\nimport mujoco.viewer\n",
        "explicit mujoco.viewer import",
    )

    text = replace_once(
        text,
        'V2741_VERSION = "2.7.4.1"\n',
        'V2741_VERSION = "2.7.4.1"\nV2742_VERSION = "2.7.4.2"\n',
        "V2-7.4.2 backend marker",
    )

    text = replace_once(
        text,
        "from NERO_V2_7_4_1_Agent import",
        "from NERO_V2_7_4_2_Agent import",
        "V2-7.4.2 Agent import rename",
    )

    old = '''        sim_ready.value = True\n        sim_error_code.value = 0\n\n        viewer_period = 1.0 / 60.0\n        last_viewer_sync = 0.0\n        next_step_wall = time.perf_counter()\n\n        with mujoco.viewer.launch_passive(model, data) as viewer:\n            while viewer.is_running() and not stop_event.is_set():\n'''
    new = '''        # V2-7.4.2: sim_ready means the MuJoCo VIEWER is actually alive, not\n        # merely that the physics model/ROS bridge initialized.  The old code\n        # set sim_ready before launch_passive(), so a viewer startup failure in\n        # the spawned child could be hidden while the Tk parent stayed alive.\n        sim_ready.value = False\n        sim_error_code.value = 0\n\n        viewer_period = 1.0 / 60.0\n        last_viewer_sync = 0.0\n        next_step_wall = time.perf_counter()\n        _viewer_ready_path = os.environ.get("NERO_VIEWER_READY_FILE", "").strip()\n\n        print(\n            "[MUJOCO VIEWER START] "\n            f"DISPLAY={os.environ.get('DISPLAY', '')!r} "\n            f"WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY', '')!r} "\n            f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '')!r}",\n            flush=True,\n        )\n\n        with mujoco.viewer.launch_passive(model, data) as viewer:\n            if not viewer.is_running():\n                raise RuntimeError("MuJoCo viewer context opened but viewer.is_running() is false")\n            # Force at least one successful GUI sync before advertising readiness.\n            viewer.sync()\n            sim_ready.value = True\n            if _viewer_ready_path:\n                try:\n                    _p = Path(_viewer_ready_path)\n                    _p.parent.mkdir(parents=True, exist_ok=True)\n                    _p.write_text(\n                        f"MUJOCO_VIEWER_READY pid={os.getpid()} time={time.time():.6f}\\n",\n                        encoding="utf-8",\n                    )\n                except Exception as _marker_exc:\n                    raise RuntimeError(\n                        f"MuJoCo viewer opened but readiness marker could not be written: {_marker_exc}"\n                    ) from _marker_exc\n            print("[MUJOCO VIEWER READY] passive viewer opened and first sync completed", flush=True)\n\n            while viewer.is_running() and not stop_event.is_set():\n'''
    text = replace_once(text, old, new, "viewer readiness handshake")

    # Make the child-process exception print enough display context to diagnose GL/desktop issues.
    old_exc = '''        print("\\n========== NERO SIMULATION ERROR ==========", flush=True)\n        traceback.print_exc()\n        print("===========================================\\n", flush=True)\n'''
    new_exc = '''        print("\\n========== NERO SIMULATION ERROR ==========", flush=True)\n        print(\n            "Viewer environment: "\n            f"DISPLAY={os.environ.get('DISPLAY', '')!r} "\n            f"WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY', '')!r} "\n            f"MUJOCO_GL={os.environ.get('MUJOCO_GL', '')!r}",\n            flush=True,\n        )\n        traceback.print_exc()\n        print("===========================================\\n", flush=True)\n'''
    text = replace_once(text, old_exc, new_exc, "viewer failure diagnostics")

    text = text.replace("NERO V2-7.4.1", "NERO V2-7.4.2")
    return text


def patch_agent(text: str) -> str:
    text = text.replace('PROJECT_VERSION = "2.7.4.1"', 'PROJECT_VERSION = "2.7.4.2"', 1)
    text = text.replace("NERO V2-7.4.1", "NERO V2-7.4.2")
    marker = "# NERO V2-7.4.2: MuJoCo viewer-startup handshake hotfix; orchestration behavior inherited from V2-7.4.1.\n"
    if marker not in text:
        text = marker + text
    return text


def patch_launcher(text: str) -> str:
    text = text.replace("NERO_V2_7_4_1_Unified_Backend.py", OUT_BACKEND)
    text = text.replace("NERO_V2_7_4_1_Agent.py", OUT_AGENT)
    text = text.replace("NERO V2-7.4.1", "NERO V2-7.4.2")
    text = text.replace("v2_7_4_1", "v2_7_4_2")

    # A concrete file written by the spawned MuJoCo child only after
    # launch_passive() enters, viewer.is_running() is true, and viewer.sync() succeeds.
    anchor = '''KIN_BACKUP="${LOG_DIR}/v2_7_4_2_kinematics_backup.yaml"\n'''
    if anchor not in text:
        # Compatibility with a launcher whose log prefix remained v2_6.
        anchor = '''KIN_BACKUP="${LOG_DIR}/v2_6_kinematics_backup.yaml"\n'''
    replacement = anchor + '''VIEWER_READY_FILE="${LOG_DIR}/v2_7_4_2_mujoco_viewer.ready"\nrm -f "$VIEWER_READY_FILE"\nexport NERO_VIEWER_READY_FILE="$VIEWER_READY_FILE"\n'''
    text = replace_once(text, anchor, replacement, "viewer marker launcher setup")

    backend_start = '''python3 -u "$BACKEND" >"$BACKEND_LOG" 2>&1 &\nBACKEND_PID=$!\nstatus OK "Backend started PID=$BACKEND_PID"\n\n'''
    handshake = backend_start + '''# V2-7.4.2: do not start MoveIt and do not claim SYSTEM READY until the\n# spawned MuJoCo child proves the graphical viewer actually opened.\nstatus INFO "Waiting for MuJoCo viewer handshake (launch_passive + first sync)"\nVIEWER_OK=0\nfor _v2742_i in $(seq 1 150); do\n    if [[ -s "$VIEWER_READY_FILE" ]]; then\n        VIEWER_OK=1\n        status OK "MuJoCo viewer handshake READY: $(cat "$VIEWER_READY_FILE")"\n        break\n    fi\n    if ! kill -0 "$BACKEND_PID" 2>/dev/null; then\n        status FAIL "Backend exited before MuJoCo viewer handshake"\n        echo "---------------- BACKEND LOG TAIL ----------------" >&2\n        tail -n 200 "$BACKEND_LOG" 2>/dev/null >&2 || true\n        echo "--------------------------------------------------" >&2\n        exit 1\n    fi\n    sleep 0.10\ndone\nif [[ "$VIEWER_OK" -ne 1 ]]; then\n    status FAIL "MuJoCo viewer did not report READY within 15 seconds"\n    echo "---------------- BACKEND LOG TAIL ----------------" >&2\n    tail -n 240 "$BACKEND_LOG" 2>/dev/null >&2 || true\n    echo "--------------------------------------------------" >&2\n    fatal "MuJoCo GUI startup failed; refusing false SYSTEM READY"\nfi\n\n'''
    text = replace_once(text, backend_start, handshake, "launcher viewer handshake wait")

    # Strengthen the existing V2-7.4.1 backend liveness gate with marker presence.
    old_gate = '''status OK "Backend remained alive through GUI startup grace period"\nstatus INFO "Close the NERO V2-7.4.2 GUI to stop Agent UI, both MoveIt instances, and MuJoCo"\n'''
    new_gate = '''if [[ ! -s "$VIEWER_READY_FILE" ]]; then\n    status FAIL "MuJoCo viewer readiness marker disappeared after SYSTEM READY"\n    echo "---------------- BACKEND LOG TAIL ----------------" >&2\n    tail -n 200 "$BACKEND_LOG" 2>/dev/null >&2 || true\n    echo "--------------------------------------------------" >&2\n    exit 1\nfi\nstatus OK "Backend and MuJoCo viewer remained alive through GUI startup grace period"\nstatus INFO "Close the NERO V2-7.4.2 GUI to stop Agent UI, both MoveIt instances, and MuJoCo"\n'''
    text = replace_once(text, old_gate, new_gate, "post-readiness viewer gate")

    return text


def write_docs(out: Path) -> None:
    (out / "README_V2_7_4_2.md").write_text(
        "# NERO V2-7.4.2 — MuJoCo Viewer Handshake Hotfix\n\n"
        "This package fixes a false-ready condition in V2-7.4.1.\n\n"
        "## Fixes\n"
        "- Explicit `import mujoco.viewer` in the spawned simulation process.\n"
        "- `sim_ready` is asserted only after `launch_passive()` enters and the first `viewer.sync()` succeeds.\n"
        "- The MuJoCo child writes a viewer-ready marker file.\n"
        "- The launcher waits for that marker before MoveIt startup and before declaring SYSTEM READY.\n"
        "- If the viewer cannot open, the launcher prints the backend traceback and DISPLAY/WAYLAND/MUJOCO_GL diagnostics.\n"
        "- V69 remains byte-for-byte frozen.\n\n"
        "## Run\n"
        "```bash\n"
        "cd ~/mujoco_ws/nero/NERO_V2_7_4_2_MuJoCo_Viewer_Handshake\n"
        "chmod +x start_nero_v2_7_4_2.sh\n"
        "./start_nero_v2_7_4_2.sh\n"
        "```\n",
        encoding="utf-8",
    )

    (out / "PATH_CHECK.txt").write_text(
        "NERO V2-7.4.2 exact runtime contract\n"
        "===================================\n"
        f"./{OUT_BACKEND}\n"
        f"./{OUT_AGENT}\n"
        f"./{CORE}\n"
        f"./{OUT_LAUNCHER}\n"
        "\nThe launcher copies these Python files into ${NERO_WS:-$HOME/mujoco_ws/nero}.\n"
        "It also requires the existing ~/mujoco_ws/nero/nero_motor.xml and AgileX/ROS workspaces.\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    src = args.source_dir.expanduser().resolve()
    out = args.output_dir.expanduser().resolve() if args.output_dir else src.parent / OUT_DIR_NAME

    required = [SRC_BACKEND, SRC_AGENT, SRC_LAUNCHER, CORE]
    missing = [name for name in required if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError("Incomplete V2-7.4.1 source runtime: " + ", ".join(missing))

    if sha256(src / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 SHA mismatch in V2-7.4.1 source")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

    backend_text = (src / SRC_BACKEND).read_text(encoding="utf-8")
    agent_text = (src / SRC_AGENT).read_text(encoding="utf-8")
    launcher_text = (src / SRC_LAUNCHER).read_text(encoding="utf-8")

    (out / OUT_BACKEND).write_text(patch_backend(backend_text), encoding="utf-8")
    (out / OUT_AGENT).write_text(patch_agent(agent_text), encoding="utf-8")
    (out / OUT_LAUNCHER).write_text(patch_launcher(launcher_text), encoding="utf-8")
    (out / OUT_LAUNCHER).chmod((out / OUT_LAUNCHER).stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    for stale in [SRC_BACKEND, SRC_AGENT, SRC_LAUNCHER, "README_V2_7_4_1.md"]:
        p = out / stale
        if p.is_file():
            p.unlink()

    write_docs(out)

    py_compile.compile(str(out / OUT_BACKEND), doraise=True)
    py_compile.compile(str(out / OUT_AGENT), doraise=True)
    subprocess.run(["bash", "-n", str(out / OUT_LAUNCHER)], check=True)

    if sha256(out / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during V2-7.4.2 build")

    b = (out / OUT_BACKEND).read_text(encoding="utf-8")
    l = (out / OUT_LAUNCHER).read_text(encoding="utf-8")
    required_backend = [
        "import mujoco.viewer",
        'V2742_VERSION = "2.7.4.2"',
        "[MUJOCO VIEWER START]",
        "[MUJOCO VIEWER READY]",
        "viewer.sync()",
        "NERO_VIEWER_READY_FILE",
        "sim_ready.value = False",
    ]
    required_launcher = [
        OUT_BACKEND,
        OUT_AGENT,
        "VIEWER_READY_FILE",
        "Waiting for MuJoCo viewer handshake",
        "refusing false SYSTEM READY",
        "BACKEND LOG TAIL",
    ]
    missing_markers = [x for x in required_backend if x not in b] + [x for x in required_launcher if x not in l]
    if missing_markers:
        raise RuntimeError(f"V2-7.4.2 static marker validation failed: {missing_markers}")

    # Ensure the old false-ready ordering no longer exists.
    old_order = "sim_ready.value = True\n        sim_error_code.value = 0\n\n        viewer_period"
    if old_order in b:
        raise RuntimeError("Old pre-viewer sim_ready=true ordering still present")

    sums = []
    for name in (OUT_BACKEND, OUT_AGENT, CORE, OUT_LAUNCHER):
        sums.append(f"{sha256(out / name)}  {name}")
    (out / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")

    print("[OK] V2-7.4.2 runtime created:", out)
    print("[OK] Explicit mujoco.viewer import present")
    print("[OK] Viewer-ready handshake inserted before MoveIt readiness")
    print("[OK] Python + Bash syntax validated")
    print("[OK] Frozen V69 SHA256:", sha256(out / CORE))


if __name__ == "__main__":
    main()
