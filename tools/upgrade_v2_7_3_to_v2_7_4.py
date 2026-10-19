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

SOURCE_BACKEND = "NERO_V2_7_1_Unified_Backend.py"
SOURCE_AGENT = "NERO_V2_7_1_Agent.py"
SOURCE_LAUNCHER = "start_nero_v2_7_3.sh"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

OUT_BACKEND = "NERO_V2_7_4_Unified_Backend.py"
OUT_AGENT = "NERO_V2_7_4_Agent.py"
OUT_LAUNCHER = "start_nero_v2_7_4.sh"
OUT_DIR_NAME = "NERO_V2_7_4_Agent_Orchestration_Hardening"


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
            "refusing to produce a partially patched V2-7.4 runtime"
        )
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if 'V273_VERSION = "2.7.3"' not in text:
        raise RuntimeError("Source backend is not the validated V2-7.3 runtime")
    if 'V274_VERSION = "2.7.4"' in text:
        raise RuntimeError("Backend already contains the V2-7.4 marker")

    # Rename the Agent import so the generated runtime has one coherent set of
    # V2-7.4 entry-point filenames.  V69 remains frozen and keeps its old name.
    if "from NERO_V2_7_1_Agent import" not in text:
        raise RuntimeError("Could not find V2-7.3 Agent import in backend")
    text = text.replace(
        "from NERO_V2_7_1_Agent import",
        "from NERO_V2_7_4_Agent import",
        1,
    )

    # Add a version marker without disturbing any inherited V2-7.1/2/3 helper
    # names.  Those names are intentionally kept because this is incremental.
    text = replace_once(
        text,
        'V273_VERSION = "2.7.3"\n',
        'V273_VERSION = "2.7.3"\nV274_VERSION = "2.7.4"\n',
        "V2-7.4 backend marker",
    )

    # GUI log styling.  V2-7.3 already gives task summaries their own colored
    # bold tags.  V2-7.4 adds one neutral bold tag for every Agent/LLM message.
    # Use substring matching because the unified logger may prepend [HH:MM:SS].
    log_anchor = '    def append_gui_log(text):\n'
    log_tag = (
        '    global_log.tag_configure(\n'
        '        "llm_message_bold", font=("Arial", 10, "bold")\n'
        '    )\n\n'
    )
    text = replace_once(text, log_anchor, log_tag + log_anchor, "LLM bold tag insertion")

    old = (
        '        elif _msg.startswith("[TASK SUMMARY RED]"):\n'
        '            _tag = "task_summary_failure"\n'
        '        if _tag:\n'
    )
    new = (
        '        elif _msg.startswith("[TASK SUMMARY RED]"):\n'
        '            _tag = "task_summary_failure"\n'
        '        elif "[LLM " in _msg:\n'
        '            _tag = "llm_message_bold"\n'
        '        if _tag:\n'
    )
    text = replace_once(text, old, new, "LLM bold routing")

    text = text.replace("NERO V2-7.3", "NERO V2-7.4")
    return text


def patch_agent(text: str) -> str:
    if "V2-7.3 TASK SUMMARY" not in text:
        raise RuntimeError("Source Agent is not the validated V2-7.3 Agent")
    if "V2-7.4 AGENT ORCHESTRATION" in text:
        raise RuntimeError("Agent already contains the V2-7.4 patch")

    # Keep the LLM responsible for semantic strategy, but make two short,
    # safety-critical transitions deterministic at the orchestration layer:
    #   (1) a verified approach is immediately followed by clamp_gripper;
    #   (2) an LLM final response is accepted when every physically pending
    #       object is already terminally blocked by deterministic planning.
    policy = '''\nV2-7.4 AGENT ORCHESTRATION:\n- VERIFIED APPROACH LOCK: if approach_planned_grasp succeeds and reaches GRASP_POSE_REACHED, do not spend another LLM turn deciding whether to clamp. The Agent orchestrator immediately invokes clamp_gripper(force_n=5.0). The existing clamp skill still owns all bystander/safety checks; a rejected clamp remains a failed grasp branch.\n- TERMINAL-AWARE COMPLETION: the plural-task guard must not force another observe/replan cycle for objects already proven terminally blocked by ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE / REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7. Only actionable pending objects may trigger a completion-guard nudge.\n- USER STOP IS A TERMINAL EVENT: an explicit Stop request is not an internal Agent failure. Finalize the episode, print a USER_ABORTED high-level summary from completed tool history, and do not raise a GUI-level RUN TASK failure solely because the user stopped it.\n'''
    text = replace_once(
        text,
        "\nOperating policy:",
        policy + "\nOperating policy:",
        "V2-7.4 orchestration policy",
    )

    # Deterministic task-finalizer state.
    old = (
        '        v273_tool_history = []\n'
        '        v273_final_text = ""\n'
        '        v273_exception_text = None\n'
    )
    new = old + (
        '        v274_terminal_objects = set()\n'
        '        v274_user_aborted = False\n'
    )
    text = replace_once(text, old, new, "V2-7.4 episode state")

    # A user Stop gets its own summary state instead of being folded into FAILED.
    old = (
        '            if v273_exception_text:\n'
        '                color = "RED"\n'
        '                status = "FAILED"\n'
        '            elif all_requested_verified or (not requested and any_terminal_verified):\n'
    )
    new = (
        '            if v274_user_aborted:\n'
        '                color = "YELLOW"\n'
        '                status = "USER_ABORTED"\n'
        '            elif v273_exception_text:\n'
        '                color = "RED"\n'
        '                status = "FAILED"\n'
        '            elif all_requested_verified or (not requested and any_terminal_verified):\n'
    )
    text = replace_once(text, old, new, "USER_ABORTED summary classification")

    old = (
        '            if not outcome:\n'
        '                outcome = "No final model text was produced."\n'
    )
    new = (
        '            if not outcome:\n'
        '                if v274_user_aborted:\n'
        '                    outcome = "Stopped by user; summary generated from completed tool history."\n'
        '                else:\n'
        '                    outcome = "No final model text was produced."\n'
    )
    text = replace_once(text, old, new, "USER_ABORTED summary outcome")

    old = (
        '                f"PHYSICAL VERIFY: {len(verified_objects)} object(s) verified | "\n'
        '                f"requested={requested or \'unspecified\'}\\n"\n'
        '                f"LAST FAILURE: {last_failure}"\n'
    )
    new = (
        '                f"PHYSICAL VERIFY: {len(verified_objects)} object(s) verified | "\n'
        '                f"requested={requested or \'unspecified\'}\\n"\n'
        '                f"TERMINAL BLOCKED: {sorted(v274_terminal_objects) or \'none\'}\\n"\n'
        '                f"LAST FAILURE: {last_failure}"\n'
    )
    text = replace_once(text, old, new, "terminal-object summary line")

    # Fix the plural-task completion guard.  The old guard blindly nudged while
    # anything remained physically in the basket, even after deterministic
    # planning had proven all those objects non-actionable.
    old = (
        '                        if pending:\n'
        '                            multi_object_guard_nudges += 1\n'
    )
    new = (
        '                        if pending:\n'
        '                            _v274_actionable_pending = [\n'
        '                                _p for _p in pending if str(_p) not in v274_terminal_objects\n'
        '                            ]\n'
        '                            if not _v274_actionable_pending:\n'
        '                                self.emit(\n'
        '                                    f"[LLM ORCHESTRATOR TERMINAL] all physically pending objects are "\n'
        '                                    f"terminally blocked: {sorted(v274_terminal_objects)}; accepting final response"\n'
        '                                )\n'
        '                                pending = []\n'
        '                            else:\n'
        '                                pending = _v274_actionable_pending\n'
        '                        if pending:\n'
        '                            multi_object_guard_nudges += 1\n'
    )
    text = replace_once(text, old, new, "terminal-aware multi-object guard")

    # Insert deterministic terminal-object bookkeeping and the atomic
    # approach->clamp transition immediately before V2-7.3 records the tool
    # result for the task-end summary.
    anchor = (
        '                    v273_tool_history.append({"tool": name, "result": result})\n'
        '                    outputs.append(\n'
    )
    addition = r'''                    # V2-7.4 terminal-object bookkeeping.
                    if isinstance(result, dict):
                        _v274_pv = result.get("physical_verification") or {}
                        _v274_reason = str(
                            _v274_pv.get("reason")
                            or result.get("reason")
                            or result.get("error")
                            or ""
                        )
                        _v274_obj = None
                        if isinstance(args, dict):
                            _v274_obj = args.get("object_name")
                        if not _v274_obj:
                            _v274_obj = result.get("object_name")
                        if not _v274_obj:
                            _gp = result.get("grasp_plan") or {}
                            if isinstance(_gp, dict):
                                _v274_obj = _gp.get("object_name")
                        if (
                            name == "plan_grasp"
                            and not bool(result.get("ok", False))
                            and _v274_obj
                            and _v274_reason in (
                                "ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE",
                                "REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7",
                            )
                        ):
                            v274_terminal_objects.add(str(_v274_obj))
                            self.emit(
                                f"[LLM ORCHESTRATOR TERMINAL] {str(_v274_obj)} latched terminal | "
                                f"reason={_v274_reason}"
                            )

                    # V2-7.4 VERIFIED APPROACH LOCK.  The model already chose the
                    # grasp candidate; after deterministic physical verification,
                    # closing is a short execution transition, not a new semantic
                    # planning decision.  The existing clamp skill still performs
                    # the full bystander/safety checks and may reject the clamp.
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
                        _v274_t_auto = time.perf_counter()
                        try:
                            _v274_clamp = self.tool_runner("clamp_gripper", {"force_n": 5.0})
                            if not isinstance(_v274_clamp, dict):
                                _v274_clamp = {
                                    "ok": bool(_v274_clamp),
                                    "result": _v274_clamp,
                                }
                        except Exception as _v274_exc:
                            _v274_clamp = {
                                "ok": False,
                                "tool": "clamp_gripper",
                                "error": str(_v274_exc),
                            }
                        self.emit(
                            f"[LLM ORCHESTRATOR AUTO-CLAMP RESULT] "
                            f"elapsed={time.perf_counter()-_v274_t_auto:.2f}s | "
                            + json.dumps(
                                _v274_clamp,
                                ensure_ascii=False,
                                separators=(",", ":"),
                            )
                        )
                        v273_tool_history.append(
                            {"tool": "clamp_gripper[auto]", "result": _v274_clamp}
                        )
                        _v274_combined = dict(result)
                        _v274_combined["orchestration_auto_followup"] = {
                            "tool": "clamp_gripper",
                            "arguments": {"force_n": 5.0},
                            "result": _v274_clamp,
                        }
                        _v274_combined["ok"] = bool(
                            result.get("ok", False) and _v274_clamp.get("ok", False)
                        )
                        if isinstance(_v274_clamp.get("state_after"), dict):
                            _v274_combined["state_after"] = _v274_clamp["state_after"]
                        if "recommended_next_actions" in _v274_clamp:
                            _v274_combined["recommended_next_actions"] = _v274_clamp[
                                "recommended_next_actions"
                            ]
                        result = _v274_combined

                    v273_tool_history.append({"tool": name, "result": result})
                    outputs.append(
'''
    text = replace_once(text, anchor, addition, "auto-clamp + terminal bookkeeping")

    # User Stop is expected control flow.  Do not propagate it to the GUI worker
    # as RUN TASK FAILED; still run the V2-7.3 finally/summary path.
    old = (
        '        except Exception as exc:\n'
        '            v273_exception_text = str(exc)\n'
        '            self.emit(f"[LLM TASK FAILED] {exc}")\n'
        '            raise\n'
        '        finally:\n'
    )
    new = (
        '        except Exception as exc:\n'
        '            if str(exc) == "LLM task stopped by user":\n'
        '                v274_user_aborted = True\n'
        '                v273_exception_text = None\n'
        '                self.emit(\n'
        '                    "[LLM TASK ABORTED] User requested stop; "\n'
        '                    "finalizing completed tool history"\n'
        '                )\n'
        '            else:\n'
        '                v273_exception_text = str(exc)\n'
        '                self.emit(f"[LLM TASK FAILED] {exc}")\n'
        '                raise\n'
        '        finally:\n'
    )
    text = replace_once(text, old, new, "user-stop terminal finalizer")

    text = text.replace('PROJECT_VERSION = "2.7.1"', 'PROJECT_VERSION = "2.7.4"', 1)
    text = text.replace("NERO V2-7.3", "NERO V2-7.4")
    return text


def patch_launcher(text: str) -> str:
    # The V2-7.3 runtime intentionally kept V2-7.1 Python filenames.  V2-7.4
    # ends that ambiguity: the package contains and launches only V2-7.4 entry
    # files.  The frozen V69 filename remains unchanged by design.
    text = text.replace(SOURCE_BACKEND, OUT_BACKEND)
    text = text.replace(SOURCE_AGENT, OUT_AGENT)
    text = text.replace("NERO V2-7.3", "NERO V2-7.4")
    text = text.replace("NERO V2-7.2", "NERO V2-7.4")
    text = text.replace("NERO V2-7.1", "NERO V2-7.4")
    text = text.replace("v2_7_3", "v2_7_4")
    text = text.replace("v2_7_2", "v2_7_4")
    text = text.replace("v2_7_1", "v2_7_4")
    return text


def write_release_docs(out: Path) -> None:
    (out / "README_V2_7_4.md").write_text(
        "# NERO V2-7.4 — Agent Orchestration Hardening\n\n"
        "V69 remains byte-for-byte frozen. V2-7.1 placement, V2-7.2 hold/wrist, "
        "and V2-7.3 late-settle/task-summary behavior remain inherited.\n\n"
        "## V2-7.4 changes\n"
        "- Verified `approach_planned_grasp` is atomically followed by "
        "`clamp_gripper(force_n=5.0)` inside the Agent orchestrator; the clamp skill "
        "still owns all bystander/safety rejection logic.\n"
        "- The multi-object completion guard nudges only actionable pending objects; "
        "terminally exhausted objects no longer cause pointless observe/replan loops.\n"
        "- User Stop is finalized as `USER_ABORTED` and still prints the high-level summary.\n"
        "- GUI log renders `[LLM ...]` records in bold; task-summary color/bold styling remains.\n"
        "- Runtime Python entry filenames are now explicitly V2-7.4.\n\n"
        "## Run\n"
        "```bash\n"
        "cd NERO_V2_7_4_Agent_Orchestration_Hardening\n"
        "chmod +x start_nero_v2_7_4.sh\n"
        "./start_nero_v2_7_4.sh\n"
        "```\n",
        encoding="utf-8",
    )

    (out / "PATH_CHECK.txt").write_text(
        "NERO V2-7.4 runtime path contract\n"
        "=================================\n"
        f"./{OUT_BACKEND}\n"
        f"./{OUT_AGENT}\n"
        f"./{CORE}\n"
        f"./{OUT_LAUNCHER}\n"
        "\nLauncher copies those three Python files into ${NERO_WS:-$HOME/mujoco_ws/nero}\n"
        "and launches the V2-7.4 backend from that established NERO workspace.\n"
        "No V2-7.1/V2-7.2/V2-7.3 Python entry file is required at runtime.\n",
        encoding="utf-8",
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir", type=Path)
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    src = args.source_dir.expanduser().resolve()
    out = (
        args.output_dir.expanduser().resolve()
        if args.output_dir
        else src.parent / OUT_DIR_NAME
    )

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
            "Frozen V69 SHA mismatch. Refusing to build V2-7.4.\n"
            f"Expected: {EXPECTED_V69_SHA256}\nActual:   {source_core_hash}"
        )

    backend_text = (src / SOURCE_BACKEND).read_text(encoding="utf-8")
    agent_text = (src / SOURCE_AGENT).read_text(encoding="utf-8")
    launcher_text = (src / SOURCE_LAUNCHER).read_text(encoding="utf-8")

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)

    backend_out = out / OUT_BACKEND
    agent_out = out / OUT_AGENT
    launcher_out = out / OUT_LAUNCHER

    backend_out.write_text(patch_backend(backend_text), encoding="utf-8")
    agent_out.write_text(patch_agent(agent_text), encoding="utf-8")
    launcher_out.write_text(patch_launcher(launcher_text), encoding="utf-8")
    launcher_out.chmod(
        launcher_out.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    )

    # Remove obsolete entry-point files/launchers from the generated V2-7.4
    # runtime so a user cannot accidentally start an older version in-place.
    obsolete = {
        SOURCE_BACKEND,
        SOURCE_AGENT,
        "start_nero_v2_7.sh",
        "start_nero_v2_7_1.sh",
        "start_nero_v2_7_2.sh",
        "start_nero_v2_7_3.sh",
    }
    for name in obsolete:
        p = out / name
        if p.is_file() and p.name != OUT_LAUNCHER:
            p.unlink()

    write_release_docs(out)

    # ------------------------------------------------------------
    # Release validation against the GENERATED runtime.
    # ------------------------------------------------------------
    py_compile.compile(str(backend_out), doraise=True)
    py_compile.compile(str(agent_out), doraise=True)
    subprocess.run(["bash", "-n", str(launcher_out)], check=True)

    final_core_hash = sha256(out / CORE)
    if final_core_hash != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during V2-7.4 generation")

    btxt = backend_out.read_text(encoding="utf-8")
    atxt = agent_out.read_text(encoding="utf-8")
    ltxt = launcher_out.read_text(encoding="utf-8")

    required_backend_markers = [
        'V274_VERSION = "2.7.4"',
        "from NERO_V2_7_4_Agent import",
        "llm_message_bold",
        'elif "[LLM " in _msg:',
    ]
    required_agent_markers = [
        "V2-7.4 AGENT ORCHESTRATION",
        "LLM ORCHESTRATOR AUTO-CLAMP",
        "v274_terminal_objects",
        "USER_ABORTED",
        "LLM TASK ABORTED",
    ]
    missing_markers = [m for m in required_backend_markers if m not in btxt]
    missing_markers += [m for m in required_agent_markers if m not in atxt]
    if missing_markers:
        raise RuntimeError(f"V2-7.4 static marker validation failed: {missing_markers}")

    # Strict path/name contract.  The launcher must reference ONLY the new
    # backend/Agent entry files plus frozen V69; no inherited Python filename.
    if OUT_BACKEND not in ltxt or OUT_AGENT not in ltxt or CORE not in ltxt:
        raise RuntimeError("V2-7.4 launcher does not reference all required runtime files")
    forbidden_launcher_names = [SOURCE_BACKEND, SOURCE_AGENT]
    stale = [name for name in forbidden_launcher_names if name in ltxt]
    if stale:
        raise RuntimeError(f"V2-7.4 launcher still contains stale entry filenames: {stale}")

    # Save hashes for the exact downloadable runtime entry files.
    sums = []
    for name in (OUT_BACKEND, OUT_AGENT, CORE, OUT_LAUNCHER):
        sums.append(f"{sha256(out / name)}  {name}")
    (out / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")

    print("[OK] NERO V2-7.4 runtime created:", out)
    print("[OK] Backend syntax:", OUT_BACKEND)
    print("[OK] Agent syntax:", OUT_AGENT)
    print("[OK] Launcher bash syntax:", OUT_LAUNCHER)
    print("[OK] Launcher path/name contract: no stale V2-7.1/V2-7.3 entry filenames")
    print("[OK] Frozen V69 SHA256:", final_core_hash)
    print("[OK] Verified approach -> deterministic auto-clamp orchestration installed")
    print("[OK] Terminal-aware plural-task completion guard installed")
    print("[OK] User Stop -> USER_ABORTED final summary installed")
    print("[OK] LLM GUI log messages -> bold tag installed")
    print("[RUN]", launcher_out)


if __name__ == "__main__":
    main()
