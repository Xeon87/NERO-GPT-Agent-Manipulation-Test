#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import py_compile
import re
import shutil
import stat
import subprocess
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
SRC_BACKEND = "NERO_V2_7_1_Unified_Backend.py"
SRC_AGENT = "NERO_V2_7_1_Agent.py"
SRC_LAUNCHER = "start_nero_v2_7_2.sh"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

OUT_BACKEND = "NERO_V2_7_5_Unified_Backend.py"
OUT_AGENT = "NERO_V2_7_5_Agent.py"
OUT_LAUNCHER = "start_nero_v2_7_5.sh"
OUT_DIR_NAME = "NERO_V2_7_5_Agent_Orchestration_From_V272"


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
    if 'V272_VERSION = "2.7.2"' not in text:
        raise RuntimeError("Source backend is not validated V2-7.2")

    text = replace_once(
        text,
        "from NERO_V2_7_1_Agent import",
        "from NERO_V2_7_5_Agent import",
        "Agent import rename",
    )

    text = replace_once(
        text,
        'V272_VERSION = "2.7.2"\n',
        'V272_VERSION = "2.7.2"\nV275_VERSION = "2.7.5"\n',
        "V2-7.5 backend marker",
    )

    old_log = '''    def append_gui_log(text):\n        global_log.insert("end", str(text).rstrip() + "\\n")\n        global_log.see("end")\n'''
    new_log = '''    def append_gui_log(text):\n        _msg = str(text).rstrip()\n        # V2-7.5: styling is configured lazily only after the Text widget exists.\n        # This changes log rendering only; it must never participate in GUI startup.\n        if not getattr(append_gui_log, "_v275_tags_ready", False):\n            try:\n                global_log.tag_configure("llm_message_bold", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_success", foreground="#138A36", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_warning", foreground="#A66B00", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_failure", foreground="#C62828", font=("Arial", 10, "bold"))\n            except Exception as _v275_style_exc:\n                try:\n                    print(f"[GUI STYLE WARN] V2-7.5 log styling unavailable: {_v275_style_exc}", flush=True)\n                except Exception:\n                    pass\n            append_gui_log._v275_tags_ready = True\n\n        _tag = None\n        if _msg.startswith("[TASK SUMMARY GREEN]"):\n            _tag = "task_summary_success"\n        elif _msg.startswith("[TASK SUMMARY YELLOW]"):\n            _tag = "task_summary_warning"\n        elif _msg.startswith("[TASK SUMMARY RED]"):\n            _tag = "task_summary_failure"\n        elif "[LLM " in _msg:\n            _tag = "llm_message_bold"\n\n        try:\n            if _tag:\n                global_log.insert("end", _msg + "\\n", _tag)\n            else:\n                global_log.insert("end", _msg + "\\n")\n        except Exception:\n            global_log.insert("end", _msg + "\\n")\n        global_log.see("end")\n'''
    text = replace_once(text, old_log, new_log, "lazy bold/color GUI log patch")

    text = text.replace(
        "NERO V2-7.2 unified runtime log.",
        "NERO V2-7.5 unified runtime log (V2-7.2 motion/startup baseline).",
        1,
    )
    return text


def patch_agent(text: str) -> str:
    if "approach_planned_grasp" not in text or "V2-7.2 placement safety additions:" not in text:
        raise RuntimeError("Source Agent is not validated V2-7.2 egg Agent")

    text = re.sub(r'PROJECT_VERSION = "[^"]+"', 'PROJECT_VERSION = "2.7.5"', text, count=1)

    policy = '''\nV2-7.5 AGENT ORCHESTRATION PATCH (V2-7.2 motion baseline):\n- VERIFIED APPROACH -> AUTO CLAMP: when approach_planned_grasp returns ok=true and state_after.stage=GRASP_POSE_REACHED, the Agent orchestration layer immediately calls clamp_gripper(force_n=5.0). Do NOT request a second approach or wait for another LLM turn before clamping. Inspect orchestration_auto_followup; if clamp succeeds, continue to lift.\n- USER STOP FINALIZATION: a GUI/user Stop is an expected terminal event, not an unhandled Agent failure. Preserve completed tool history, emit [LLM TASK ABORTED], then always print a high-level task summary after [LLM TASK END].\n- This patch does not change V69, MoveIt, BioIK/TRAC, MuJoCo dynamics, grasp geometry, hold continuity, or V2-7.2 wrist planning.\n'''
    text = replace_once(text, "\nOperating policy:", policy + "\nOperating policy:", "V2-7.5 policy insertion")

    old_state = '''        t_task = time.perf_counter()\n        tool_count = 0\n        multi_object_guard_nudges = 0\n\n        try:\n'''
    new_state = '''        t_task = time.perf_counter()\n        tool_count = 0\n        multi_object_guard_nudges = 0\n        v275_tool_history = []\n        v275_final_text = ""\n        v275_exception_text = None\n        v275_user_aborted = False\n\n        try:\n'''
    text = replace_once(text, old_state, new_state, "episode summary state")

    old_final = '''                    self._last_final_text = text\n                    self.emit(\n                        f"[LLM TASK COMPLETE] tool_steps={tool_count} | "\n'''
    new_final = '''                    self._last_final_text = text\n                    v275_final_text = text\n                    self.emit(\n                        f"[LLM TASK COMPLETE] tool_steps={tool_count} | "\n'''
    text = replace_once(text, old_final, new_final, "capture final text")

    output_anchor = '''                    outputs.append(\n                        {\n                            "type": "function_call_output",\n                            "call_id": call_id,\n                            "output": json.dumps(result, ensure_ascii=False),\n                        }\n                    )\n'''
    orchestration = r'''                    # V2-7.5 deterministic short transition: a physically verified
                    # grasp-pose arrival should not spend another LLM turn deciding whether
                    # to close the fingers. The existing clamp skill still owns safety and
                    # contact validation; failed clamp remains a real failed tool result.
                    if (
                        not dry_run
                        and len(calls) == 1
                        and name == "approach_planned_grasp"
                        and isinstance(result, dict)
                        and bool(result.get("ok", False))
                        and str((result.get("state_after") or {}).get("stage", ""))
                            == "GRASP_POSE_REACHED"
                    ):
                        self.emit(
                            "[LLM ORCHESTRATOR AUTO-CLAMP] verified approach -> "
                            "clamp_gripper(force_n=5.0) without another LLM round-trip"
                        )
                        _v275_t_auto = time.perf_counter()
                        tool_count += 1
                        try:
                            _v275_clamp = self.tool_runner("clamp_gripper", {"force_n": 5.0})
                            if not isinstance(_v275_clamp, dict):
                                _v275_clamp = {"ok": bool(_v275_clamp), "result": _v275_clamp}
                        except Exception as _v275_exc:
                            _v275_clamp = {
                                "ok": False,
                                "tool": "clamp_gripper",
                                "error": str(_v275_exc),
                            }
                        self.emit(
                            f"[LLM ORCHESTRATOR AUTO-CLAMP RESULT] "
                            f"elapsed={time.perf_counter()-_v275_t_auto:.2f}s | "
                            + json.dumps(_v275_clamp, ensure_ascii=False, separators=(",", ":"))
                        )
                        v275_tool_history.append(
                            {"tool": "clamp_gripper[auto]", "result": _v275_clamp}
                        )
                        _v275_combined = dict(result)
                        _v275_combined["orchestration_auto_followup"] = {
                            "tool": "clamp_gripper",
                            "arguments": {"force_n": 5.0},
                            "result": _v275_clamp,
                        }
                        _v275_combined["ok"] = bool(
                            result.get("ok", False) and _v275_clamp.get("ok", False)
                        )
                        if isinstance(_v275_clamp.get("state_after"), dict):
                            _v275_combined["state_after"] = _v275_clamp["state_after"]
                        if "physical_verification" in _v275_clamp:
                            _v275_combined["clamp_physical_verification"] = _v275_clamp[
                                "physical_verification"
                            ]
                        if "warnings" in _v275_clamp:
                            _v275_combined.setdefault("warnings", [])
                            _v275_combined["warnings"] = list(_v275_combined.get("warnings") or []) + list(
                                _v275_clamp.get("warnings") or []
                            )
                        if "recommended_next_actions" in _v275_clamp:
                            _v275_combined["recommended_next_actions"] = _v275_clamp[
                                "recommended_next_actions"
                            ]
                        result = _v275_combined

                    v275_tool_history.append({"tool": name, "result": result})
                    outputs.append(
                        {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps(result, ensure_ascii=False),
                        }
                    )
'''
    text = replace_once(text, output_anchor, orchestration, "auto-clamp insertion")

    old_tail = '''        except Exception as exc:\n            self.emit(f"[LLM TASK FAILED] {exc}")\n            raise\n        finally:\n            with self._lock:\n                self._running = False\n            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")\n'''
    new_tail = r'''        except Exception as exc:
            if str(exc) == "LLM task stopped by user":
                v275_user_aborted = True
                self.emit(
                    "[LLM TASK ABORTED] User requested stop; finalizing completed tool history"
                )
            else:
                v275_exception_text = str(exc)
                self.emit(f"[LLM TASK FAILED] {exc}")
                raise
        finally:
            with self._lock:
                self._running = False
            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")

            # V2-7.5 deterministic task-end summary. This is deliberately not an
            # extra LLM request, so it also works when the user aborts the Agent.
            _v275_failed = []
            _v275_warning_count = 0
            _v275_verified = []
            _v275_last_failure = "none"
            for _v275_rec in v275_tool_history:
                _v275_name = str(_v275_rec.get("tool", "unknown"))
                _v275_result = _v275_rec.get("result")
                if not isinstance(_v275_result, dict):
                    continue
                if not bool(_v275_result.get("ok", False)):
                    _v275_failed.append(_v275_name)
                    _v275_last_failure = str(
                        _v275_result.get("error")
                        or (_v275_result.get("physical_verification") or {}).get("reason")
                        or _v275_result.get("reason")
                        or _v275_name
                    )
                _v275_warning_count += len(_v275_result.get("warnings") or [])
                if (
                    "verify_object" in _v275_name
                    and bool(_v275_result.get("ok", False))
                ):
                    _v275_verified.append(_v275_name)

            if v275_user_aborted:
                _v275_color = "YELLOW"
                _v275_status = "USER_ABORTED"
                _v275_outcome = (
                    "Stopped by user; completed tool history was preserved and summarized."
                )
            elif v275_exception_text:
                _v275_color = "RED"
                _v275_status = "FAILED"
                _v275_outcome = v275_exception_text
            elif _v275_failed:
                _v275_color = "RED"
                _v275_status = "FAILED"
                _v275_outcome = v275_final_text or "One or more deterministic tools failed."
            elif _v275_warning_count:
                _v275_color = "YELLOW"
                _v275_status = "COMPLETED_WITH_WARNINGS"
                _v275_outcome = v275_final_text or "Task ended with warnings/recovery events."
            elif _v275_verified:
                _v275_color = "GREEN"
                _v275_status = "SUCCESS"
                _v275_outcome = v275_final_text or "Placement verification succeeded."
            else:
                _v275_color = "YELLOW"
                _v275_status = "ENDED_UNVERIFIED"
                _v275_outcome = v275_final_text or "Agent ended without a final physical placement verification."

            _v275_summary = (
                f"[TASK SUMMARY {_v275_color}]\n"
                f"STATUS: {_v275_status}\n"
                f"TASK: {task}\n"
                f"OUTCOME: {_v275_outcome}\n"
                f"TOOLS: {len(v275_tool_history)} recorded | failed={len(_v275_failed)} | "
                f"warnings={_v275_warning_count}\n"
                f"PHYSICAL VERIFY: {len(_v275_verified)} final verification tool(s) succeeded\n"
                f"LAST FAILURE: {_v275_last_failure}"
            )
            self.emit(_v275_summary)
'''
    text = replace_once(text, old_tail, new_tail, "user-stop + task summary finalizer")

    marker = "# NERO V2-7.5: Agent orchestration hardening on validated V2-7.2 motion/startup baseline.\n"
    if marker not in text:
        text = marker + text
    return text


def patch_launcher(text: str) -> str:
    # Keep V2-7.2 process startup, RViz launch, MoveIt order, cleanup, and readiness
    # checks byte-for-byte except for the package entry-point filenames/version label.
    text = replace_once(text, "NERO_V2_7_1_Unified_Backend.py", OUT_BACKEND, "launcher backend path #1")
    # The backend filename appears in BACKEND= and packaged loop, so replace remaining occurrence.
    text = text.replace("NERO_V2_7_1_Unified_Backend.py", OUT_BACKEND)
    text = text.replace("NERO_V2_7_1_Agent.py", OUT_AGENT)
    text = text.replace("NERO V2-7.2", "NERO V2-7.5")
    return text


def write_docs(out: Path) -> None:
    (out / "README_V2_7_5.md").write_text(
        "# NERO V2-7.5 — Agent Orchestration Hardening from V2-7.2\n\n"
        "V2-7.5 deliberately returns to the validated V2-7.2 motion/startup baseline.\n\n"
        "Changes only:\n"
        "1. successful approach_planned_grasp -> deterministic auto clamp_gripper;\n"
        "2. user Stop -> deterministic high-level task summary;\n"
        "3. LLM log messages are bold; task summaries are green/yellow/red.\n\n"
        "Not changed: V69, MuJoCo startup, RViz startup, BioIK/TRAC startup, MoveIt launch order, hold continuity, wrist planning.\n\n"
        "Run from the extracted package directory:\n"
        "  chmod +x start_nero_v2_7_5.sh\n"
        "  ./start_nero_v2_7_5.sh\n",
        encoding="utf-8",
    )
    (out / "PATH_CHECK.txt").write_text(
        "NERO V2-7.5 exact package/runtime contract\n"
        "=========================================\n"
        f"./{OUT_BACKEND}\n"
        f"./{OUT_AGENT}\n"
        f"./{CORE}\n"
        f"./{OUT_LAUNCHER}\n"
        "./README_V2_7_5.md\n"
        "./SHA256SUMS.txt\n\n"
        "Launcher baseline: V2-7.2.\n"
        "Launcher copies the three Python files above into ${NERO_WS:-$HOME/mujoco_ws/nero}.\n"
        "Existing ~/mujoco_ws/nero/nero_motor.xml and existing ROS/AgileX workspaces are reused exactly as V2-7.2 does.\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, required=True)
    args = ap.parse_args()

    src = args.source_dir.expanduser().resolve()
    out = args.output_dir.expanduser().resolve()

    required = [SRC_BACKEND, SRC_AGENT, SRC_LAUNCHER, CORE]
    missing = [name for name in required if not (src / name).is_file()]
    if missing:
        raise FileNotFoundError("Incomplete validated V2-7.2 source runtime: " + ", ".join(missing))
    if sha256(src / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 SHA mismatch in V2-7.2 source")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))

    (out / OUT_BACKEND).write_text(
        patch_backend((src / SRC_BACKEND).read_text(encoding="utf-8")), encoding="utf-8"
    )
    (out / OUT_AGENT).write_text(
        patch_agent((src / SRC_AGENT).read_text(encoding="utf-8")), encoding="utf-8"
    )
    (out / OUT_LAUNCHER).write_text(
        patch_launcher((src / SRC_LAUNCHER).read_text(encoding="utf-8")), encoding="utf-8"
    )
    (out / OUT_LAUNCHER).chmod(
        (out / OUT_LAUNCHER).stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    )

    # Remove stale entry points so extracted package has exactly one obvious runtime.
    for stale in [SRC_BACKEND, SRC_AGENT, SRC_LAUNCHER]:
        p = out / stale
        if p.is_file():
            p.unlink()

    write_docs(out)

    py_compile.compile(str(out / OUT_BACKEND), doraise=True)
    py_compile.compile(str(out / OUT_AGENT), doraise=True)
    subprocess.run(["bash", "-n", str(out / OUT_LAUNCHER)], check=True)

    if sha256(out / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during V2-7.5 build")

    b = (out / OUT_BACKEND).read_text(encoding="utf-8")
    a = (out / OUT_AGENT).read_text(encoding="utf-8")
    l = (out / OUT_LAUNCHER).read_text(encoding="utf-8")

    required_backend = [
        'V275_VERSION = "2.7.5"',
        "from NERO_V2_7_5_Agent import",
        "_v275_tags_ready",
        'elif "[LLM " in _msg:',
        "task_summary_success",
        "task_summary_warning",
        "task_summary_failure",
    ]
    required_agent = [
        'PROJECT_VERSION = "2.7.5"',
        "V2-7.5 AGENT ORCHESTRATION PATCH",
        "LLM ORCHESTRATOR AUTO-CLAMP",
        "clamp_gripper[auto]",
        "v275_user_aborted",
        "LLM TASK ABORTED",
        "TASK SUMMARY",
    ]
    required_launcher = [
        OUT_BACKEND,
        OUT_AGENT,
        CORE,
        'USE_RVIZ="${USE_RVIZ:-true}"',
        'use_rviz:="${USE_RVIZ}"',
        "Starting NERO V2-7.5 backend + Tk GUI + MuJoCo",
        "Primary BioIK MoveIt launch PID",
        "TRAC reference MoveIt launch PID",
    ]
    missing_markers = [x for x in required_backend if x not in b]
    missing_markers += [x for x in required_agent if x not in a]
    missing_markers += [x for x in required_launcher if x not in l]
    if missing_markers:
        raise RuntimeError(f"V2-7.5 static marker validation failed: {missing_markers}")

    stale_refs = [SRC_BACKEND, SRC_AGENT]
    stale_found = [x for x in stale_refs if x in l]
    if stale_found:
        raise RuntimeError(f"V2-7.5 launcher still contains stale entry refs: {stale_found}")

    # Verify no accidental V2-7.3/7.4 startup handshake code leaked in.
    forbidden = [
        "NERO_VIEWER_READY_FILE",
        "Waiting for MuJoCo viewer handshake",
        "Post-readiness GUI/backend liveness check",
    ]
    leaked = [x for x in forbidden if x in l or x in b]
    if leaked:
        raise RuntimeError(f"Non-V2-7.2 startup code leaked into V2-7.5: {leaked}")

    sums = []
    for name in [OUT_BACKEND, OUT_AGENT, CORE, OUT_LAUNCHER]:
        sums.append(f"{sha256(out / name)}  {name}")
    (out / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")

    print("[OK] NERO V2-7.5 runtime created:", out)
    print("[OK] Baseline: clean-room V2-7.2")
    print("[OK] Auto-clamp orchestration installed")
    print("[OK] User-stop summary installed")
    print("[OK] LLM bold + summary color GUI tags installed lazily")
    print("[OK] V2-7.2 MuJoCo/RViz/MoveIt startup structure preserved")
    print("[OK] Frozen V69 SHA256:", EXPECTED_V69_SHA256)
    print("[RUN]", out / OUT_LAUNCHER)


if __name__ == "__main__":
    main()
