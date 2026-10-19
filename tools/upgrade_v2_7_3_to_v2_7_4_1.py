#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import py_compile
import shutil
import stat
import subprocess
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"

SOURCE_BACKEND = "NERO_V2_7_1_Unified_Backend.py"
SOURCE_AGENT = "NERO_V2_7_1_Agent.py"
SOURCE_LAUNCHER = "start_nero_v2_7_3.sh"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

OUT_BACKEND = "NERO_V2_7_4_1_Unified_Backend.py"
OUT_AGENT = "NERO_V2_7_4_1_Agent.py"
OUT_LAUNCHER = "start_nero_v2_7_4_1.sh"
OUT_DIR_NAME = "NERO_V2_7_4_1_GUI_Startup_Hotfix"

HERE = Path(__file__).resolve().parent
BASE_UPGRADER = HERE / "upgrade_v2_7_3_to_v2_7_4.py"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def replace_once(text: str, old: str, new: str, label: str) -> str:
    n = text.count(old)
    if n != 1:
        raise RuntimeError(
            f"{label}: expected exactly one anchor, found {n}; "
            "refusing to create a partial V2-7.4.1 runtime"
        )
    return text.replace(old, new, 1)


def load_base_upgrader():
    if not BASE_UPGRADER.is_file():
        raise FileNotFoundError(f"Base V2-7.4 upgrader missing: {BASE_UPGRADER}")
    spec = importlib.util.spec_from_file_location("nero_v274_base_upgrader", BASE_UPGRADER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load base upgrader: {BASE_UPGRADER}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


BASE = load_base_upgrader()


def patch_backend(text: str) -> str:
    """Apply V2-7.4 behavior, then make GUI styling startup-safe.

    V2-7.4 configured the LLM bold tag eagerly during GUI construction.  That
    code was syntactically valid but the user's 2026-10-18 run showed the backend
    exiting immediately after launcher readiness.  V2-7.4.1 removes the eager
    styling side effect: the tag is configured lazily inside append_gui_log(),
    after the Text widget definitely exists, and styling failure is non-fatal.
    """
    text = BASE.patch_backend(text)

    text = replace_once(
        text,
        "from NERO_V2_7_4_Agent import",
        "from NERO_V2_7_4_1_Agent import",
        "V2-7.4.1 Agent import rename",
    )

    text = replace_once(
        text,
        'V274_VERSION = "2.7.4"\n',
        'V274_VERSION = "2.7.4"\nV2741_VERSION = "2.7.4.1"\n',
        "V2-7.4.1 backend marker",
    )

    eager = (
        '    global_log.tag_configure(\n'
        '        "llm_message_bold", font=("Arial", 10, "bold")\n'
        '    )\n\n'
    )
    text = replace_once(text, eager, "", "remove eager LLM bold-tag configuration")

    anchor = (
        '    def append_gui_log(text):\n'
        '        _msg = str(text).rstrip()\n'
    )
    replacement = (
        '    def append_gui_log(text):\n'
        '        _msg = str(text).rstrip()\n'
        '        # V2-7.4.1: configure the optional LLM bold style only after the\n'
        '        # Tk Text widget exists.  Styling must NEVER be able to abort GUI startup.\n'
        '        if not getattr(append_gui_log, "_v2741_bold_tag_ready", False):\n'
        '            try:\n'
        '                global_log.tag_configure(\n'
        '                    "llm_message_bold", font=("Arial", 10, "bold")\n'
        '                )\n'
        '            except Exception as _v2741_style_exc:\n'
        '                try:\n'
        '                    print(f"[GUI STYLE WARN] LLM bold tag unavailable: {_v2741_style_exc}")\n'
        '                except Exception:\n'
        '                    pass\n'
        '            append_gui_log._v2741_bold_tag_ready = True\n'
    )
    text = replace_once(text, anchor, replacement, "lazy LLM bold-tag insertion")

    text = text.replace("NERO V2-7.4", "NERO V2-7.4.1")
    return text


def patch_agent(text: str) -> str:
    text = BASE.patch_agent(text)
    text = text.replace('PROJECT_VERSION = "2.7.4"', 'PROJECT_VERSION = "2.7.4.1"', 1)
    text = text.replace("NERO V2-7.4", "NERO V2-7.4.1")
    marker = '# NERO V2-7.4.1 GUI-startup hotfix: Agent orchestration behavior inherited from V2-7.4.\n'
    if marker not in text:
        text = marker + text
    return text


def patch_launcher(text: str) -> str:
    text = BASE.patch_launcher(text)
    text = text.replace("NERO_V2_7_4_Unified_Backend.py", OUT_BACKEND)
    text = text.replace("NERO_V2_7_4_Agent.py", OUT_AGENT)
    text = text.replace("NERO V2-7.4", "NERO V2-7.4.1")
    text = text.replace("v2_7_4", "v2_7_4_1")

    close_anchor = (
        'status INFO "Close the NERO V2-7.4.1 GUI to stop Agent UI, both MoveIt instances, and MuJoCo"\n'
        'wait "$BACKEND_PID"\n'
    )
    close_replacement = r'''# V2-7.4.1 GUI/backend liveness gate.
# The launcher must not declare success and immediately disappear again.  Keep
# the backend alive for a short post-readiness grace period; if it exits, print
# the backend traceback directly in the terminal before cleanup.
status INFO "Post-readiness GUI/backend liveness check (3.0s)"
for _v2741_i in $(seq 1 15); do
    if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
        status FAIL "Backend exited after system readiness; GUI did not remain alive"
        echo "---------------- BACKEND LOG TAIL ----------------" >&2
        tail -n 160 "$BACKEND_LOG" 2>/dev/null >&2 || true
        echo "--------------------------------------------------" >&2
        exit 1
    fi
    sleep 0.2
done
status OK "Backend remained alive through GUI startup grace period"
status INFO "Close the NERO V2-7.4.1 GUI to stop Agent UI, both MoveIt instances, and MuJoCo"

set +e
wait "$BACKEND_PID"
BACKEND_RC=$?
set -e
if [[ "$BACKEND_RC" -ne 0 ]]; then
    status FAIL "Backend exited with code $BACKEND_RC"
    echo "---------------- BACKEND LOG TAIL ----------------" >&2
    tail -n 160 "$BACKEND_LOG" 2>/dev/null >&2 || true
    echo "--------------------------------------------------" >&2
    exit "$BACKEND_RC"
fi
'''
    text = replace_once(text, close_anchor, close_replacement, "GUI/backend liveness gate")
    return text


def write_release_docs(out: Path) -> None:
    (out / "README_V2_7_4_1.md").write_text(
        "# NERO V2-7.4.1 — GUI Startup Hotfix\n\n"
        "This is a startup-only hotfix over V2-7.4. V69 stays byte-for-byte frozen.\n\n"
        "## Kept from V2-7.4\n"
        "- verified approach -> deterministic Agent auto-clamp\n"
        "- terminal-aware plural-task completion guard\n"
        "- USER_ABORTED task-end summary\n"
        "- LLM log messages rendered bold\n\n"
        "## V2-7.4.1 fix\n"
        "- LLM bold Text-tag configuration is lazy and non-fatal, so GUI styling cannot kill startup.\n"
        "- launcher verifies the backend remains alive for 3 seconds after SYSTEM READY.\n"
        "- if backend exits, the launcher prints the last 160 backend-log lines automatically.\n"
        "- generated runtime entry names and launcher paths are validated before execution.\n\n"
        "## Run\n"
        "```bash\n"
        "cd ~/mujoco_ws/nero/NERO_V2_7_4_1_GUI_Startup_Hotfix\n"
        "chmod +x start_nero_v2_7_4_1.sh\n"
        "./start_nero_v2_7_4_1.sh\n"
        "```\n",
        encoding="utf-8",
    )

    (out / "PATH_CHECK.txt").write_text(
        "NERO V2-7.4.1 generated runtime path contract\n"
        "=============================================\n"
        f"./{OUT_BACKEND}\n"
        f"./{OUT_AGENT}\n"
        f"./{CORE}\n"
        f"./{OUT_LAUNCHER}\n"
        "\nThe launcher copies these three Python files into ${NERO_WS:-$HOME/mujoco_ws/nero}\n"
        "before starting the backend. No V2-7.1/V2-7.2/V2-7.3/V2-7.4 Python entry file\n"
        "is required by the generated launcher.\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    src = args.source_dir.expanduser().resolve()
    out = args.output_dir.expanduser().resolve() if args.output_dir else src.parent / OUT_DIR_NAME

    required = [SOURCE_BACKEND, SOURCE_AGENT, CORE, SOURCE_LAUNCHER]
    missing = [name for name in required if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError(
            "Validated V2-7.3 runtime is incomplete. Missing:\n  - "
            + "\n  - ".join(str(src / x) for x in missing)
        )

    source_core_hash = sha256(src / CORE)
    if source_core_hash != EXPECTED_V69_SHA256:
        raise RuntimeError(
            "Frozen V69 SHA mismatch. Refusing to build V2-7.4.1.\n"
            f"Expected: {EXPECTED_V69_SHA256}\nActual:   {source_core_hash}"
        )

    backend_text = (src / SOURCE_BACKEND).read_text(encoding="utf-8")
    agent_text = (src / SOURCE_AGENT).read_text(encoding="utf-8")
    launcher_text = (src / SOURCE_LAUNCHER).read_text(encoding="utf-8")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

    backend_out = out / OUT_BACKEND
    agent_out = out / OUT_AGENT
    launcher_out = out / OUT_LAUNCHER

    backend_out.write_text(patch_backend(backend_text), encoding="utf-8")
    agent_out.write_text(patch_agent(agent_text), encoding="utf-8")
    launcher_out.write_text(patch_launcher(launcher_text), encoding="utf-8")
    launcher_out.chmod(launcher_out.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    obsolete = {
        SOURCE_BACKEND,
        SOURCE_AGENT,
        "NERO_V2_7_4_Unified_Backend.py",
        "NERO_V2_7_4_Agent.py",
        "start_nero_v2_7.sh",
        "start_nero_v2_7_1.sh",
        "start_nero_v2_7_2.sh",
        "start_nero_v2_7_3.sh",
        "start_nero_v2_7_4.sh",
    }
    for name in obsolete:
        p = out / name
        if p.is_file() and p.name != OUT_LAUNCHER:
            p.unlink()

    write_release_docs(out)

    # Static release validation against the generated runtime.
    py_compile.compile(str(backend_out), doraise=True)
    py_compile.compile(str(agent_out), doraise=True)
    subprocess.run(["bash", "-n", str(launcher_out)], check=True)

    final_core_hash = sha256(out / CORE)
    if final_core_hash != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during V2-7.4.1 generation")

    btxt = backend_out.read_text(encoding="utf-8")
    atxt = agent_out.read_text(encoding="utf-8")
    ltxt = launcher_out.read_text(encoding="utf-8")

    required_backend_markers = [
        'V2741_VERSION = "2.7.4.1"',
        "from NERO_V2_7_4_1_Agent import",
        "_v2741_bold_tag_ready",
        'elif "[LLM " in _msg:',
    ]
    required_agent_markers = [
        "V2-7.4 AGENT ORCHESTRATION",
        "LLM ORCHESTRATOR AUTO-CLAMP",
        "v274_terminal_objects",
        "USER_ABORTED",
        "LLM TASK ABORTED",
    ]
    required_launcher_markers = [
        OUT_BACKEND,
        OUT_AGENT,
        CORE,
        "Post-readiness GUI/backend liveness check (3.0s)",
        "BACKEND LOG TAIL",
    ]

    missing_markers = [m for m in required_backend_markers if m not in btxt]
    missing_markers += [m for m in required_agent_markers if m not in atxt]
    missing_markers += [m for m in required_launcher_markers if m not in ltxt]
    if missing_markers:
        raise RuntimeError(f"V2-7.4.1 static marker validation failed: {missing_markers}")

    eager = 'global_log.tag_configure(\n        "llm_message_bold"'
    if eager in btxt and btxt.find(eager) < btxt.find("def append_gui_log"):
        raise RuntimeError("V2-7.4.1 still contains eager pre-callback LLM bold configuration")

    stale = [SOURCE_BACKEND, SOURCE_AGENT, "NERO_V2_7_4_Unified_Backend.py", "NERO_V2_7_4_Agent.py"]
    stale_found = [name for name in stale if name in ltxt]
    if stale_found:
        raise RuntimeError(f"V2-7.4.1 launcher still contains stale entry filenames: {stale_found}")

    sums = []
    for name in (OUT_BACKEND, OUT_AGENT, CORE, OUT_LAUNCHER):
        sums.append(f"{sha256(out / name)}  {name}")
    (out / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")

    print("[OK] NERO V2-7.4.1 runtime created:", out)
    print("[OK] Backend syntax:", OUT_BACKEND)
    print("[OK] Agent syntax:", OUT_AGENT)
    print("[OK] Launcher bash syntax:", OUT_LAUNCHER)
    print("[OK] Lazy/non-fatal LLM bold styling installed")
    print("[OK] 3-second post-readiness backend liveness gate installed")
    print("[OK] Backend crash -> terminal log-tail diagnostics installed")
    print("[OK] Frozen V69 SHA256:", final_core_hash)
    print("[RUN]", launcher_out)


if __name__ == "__main__":
    main()
