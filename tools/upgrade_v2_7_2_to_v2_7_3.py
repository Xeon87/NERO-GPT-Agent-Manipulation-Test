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
BACKEND = "NERO_V2_6_Unified_Backend.py"
AGENT = "NERO_V2_6_Agent.py"
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
        raise RuntimeError(
            f"{label}: expected exactly one anchor, found {n}; "
            "refusing to create a partial V2-7.3 runtime"
        )
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if 'V272_VERSION = "2.7.2"' not in text:
        raise RuntimeError(
            "Source backend is not the validated V2-7.2 runtime: "
            "missing V272_VERSION marker"
        )
    if 'V273_VERSION = "2.7.3"' in text:
        raise RuntimeError("Backend already contains V2-7.3 marker")

    # ------------------------------------------------------------------
    # 1) The V2-7.2 run proved the original 1.8 s wrapper settle window is
    #    too short for the MuJoCo dynamics to converge after ros2_control
    #    reports success.  V69 itself stays frozen.  Only the semantic wrapper
    #    waits longer before declaring a physical tracking mismatch.
    # ------------------------------------------------------------------
    old = """            settle_deadline = time.time() + 1.8\n"""
    new = """            # V2-7.3: controller success can precede final MuJoCo settling.\n            # The 2026-10-18 V2-7.2 run showed a false 177.9 mm TCP failure that\n            # entered tolerance roughly one second AFTER the old 1.8 s window expired.\n            # Keep the gate OFF and do not re-plan; only observe passive settling.\n            settle_deadline = time.time() + 4.5\n            skill_core.emit(\n                f\"[V2-7.3 LATE SETTLE WATCH] {label} | controller verify was early; \"\n                f\"holding current command state and rechecking TCP for up to 4.5s\"\n            )\n"""
    text = replace_once(text, old, new, "extend late-settle window")

    old = """                    skill_core.emit(\n                        f\"[V2-6 LATE SETTLE ACCEPT] {label} | TCP reached tolerance on recheck: \"\n                        f\"{d2*1000.0:.1f}mm <= {tol_m*1000.0:.1f}mm\"\n                    )\n                    break\n"""
    new = """                    warnings.append(\n                        \"LATE_SETTLE_RECOVERED: ros2_control/MuJoCo convergence arrived after the \"\n                        \"initial physical-joint verification timeout; no alternate grasp plan was executed.\"\n                    )\n                    skill_core.emit(\n                        f\"[V2-7.3 LATE SETTLE ACCEPT] {label} | TCP reached tolerance on recheck: \"\n                        f\"{d2*1000.0:.1f}mm <= {tol_m*1000.0:.1f}mm\"\n                    )\n                    break\n"""
    text = replace_once(text, old, new, "late-settle warning marker")

    # ------------------------------------------------------------------
    # 2) A physical tracking timeout is an execution-layer problem, NOT proof
    #    that the selected grasp geometry is bad.  Do not mark the candidate
    #    exhausted and do not tell the LLM to select a second candidate.
    # ------------------------------------------------------------------
    old = """            ok = bool(cart_ok and object_move <= 0.005)\n            if not ok:\n                V27_GRASP_RUNTIME.setdefault(\"tried\", {}).setdefault(obj, set()).add(cid)\n                verification.setdefault(\"reason\", \"PREGRASP_CANDIDATE_EXECUTION_FAILED\")\n                return _v24_result(\n                    name, False, contract, state_before, verification, warnings,\n                    {\"object_name\": obj, \"candidate_id\": cid, \"elapsed_s\": round(time.perf_counter()-t0,3)},\n                    next_stage=None,\n                    recommended_next_actions=[\"observe_scene\", \"plan_grasp\"],\n                )\n"""
    new = """            ok = bool(cart_ok and object_move <= 0.005)\n            if not ok:\n                _tracking_error = str(verification.get(\"legacy_joint_verify_error\", \"\"))\n                _pure_tracking_timeout = bool(\n                    \"PHYSICAL TRACKING MISMATCH\" in _tracking_error\n                    and int(verification.get(\"safety_trip_after\", 0))\n                        == int(verification.get(\"safety_trip_before\", 0))\n                    and object_move <= 0.005\n                )\n                if _pure_tracking_timeout:\n                    # Do NOT poison the grasp-candidate memory.  The geometry was\n                    # feasible; only physical convergence failed the extended wait.\n                    verification[\"reason\"] = \"PREGRASP_PHYSICAL_TRACKING_TIMEOUT\"\n                    verification[\"candidate_marked_tried\"] = False\n                    warnings.append(\n                        \"Execution-layer tracking did not converge within the extended settle window. \"\n                        \"Do not switch grasp candidates: stop this task and diagnose controller/dynamics timing.\"\n                    )\n                    skill_core.emit(\n                        f\"[V2-7.3 PREGRASP TRACKING ABORT] {obj}/{cid} | \"\n                        \"candidate geometry remains valid; alternate grasp selection is blocked\"\n                    )\n                    return _v24_result(\n                        name, False, contract, state_before, verification, warnings,\n                        {\"object_name\": obj, \"candidate_id\": cid, \"elapsed_s\": round(time.perf_counter()-t0,3)},\n                        next_stage=None,\n                        recommended_next_actions=[\"stop_task_and_report_motion_tracking_failure\"],\n                    )\n\n                V27_GRASP_RUNTIME.setdefault(\"tried\", {}).setdefault(obj, set()).add(cid)\n                verification.setdefault(\"reason\", \"PREGRASP_CANDIDATE_EXECUTION_FAILED\")\n                verification[\"candidate_marked_tried\"] = True\n                return _v24_result(\n                    name, False, contract, state_before, verification, warnings,\n                    {\"object_name\": obj, \"candidate_id\": cid, \"elapsed_s\": round(time.perf_counter()-t0,3)},\n                    next_stage=None,\n                    recommended_next_actions=[\"observe_scene\", \"plan_grasp\"],\n                )\n"""
    text = replace_once(text, old, new, "tracking-vs-grasp failure split")

    # ------------------------------------------------------------------
    # 3) Colored high-level summaries.  The Agent emits one multi-line record
    #    after [LLM TASK END].  Tk applies a color tag to that entire record.
    # ------------------------------------------------------------------
    old = """    def append_gui_log(text):\n        global_log.insert(\"end\", str(text).rstrip() + \"\\n\")\n        global_log.see(\"end\")\n"""
    new = """    # V2-7.3 task-end high-level summary colors.\n    global_log.tag_configure(\n        \"task_summary_success\", foreground=\"#138A36\", font=(\"Arial\", 10, \"bold\")\n    )\n    global_log.tag_configure(\n        \"task_summary_warning\", foreground=\"#A66B00\", font=(\"Arial\", 10, \"bold\")\n    )\n    global_log.tag_configure(\n        \"task_summary_failure\", foreground=\"#C62828\", font=(\"Arial\", 10, \"bold\")\n    )\n\n    def append_gui_log(text):\n        _msg = str(text).rstrip()\n        _tag = None\n        if _msg.startswith(\"[TASK SUMMARY GREEN]\"):\n            _tag = \"task_summary_success\"\n        elif _msg.startswith(\"[TASK SUMMARY YELLOW]\"):\n            _tag = \"task_summary_warning\"\n        elif _msg.startswith(\"[TASK SUMMARY RED]\"):\n            _tag = \"task_summary_failure\"\n        if _tag:\n            global_log.insert(\"end\", _msg + \"\\n\", _tag)\n        else:\n            global_log.insert(\"end\", _msg + \"\\n\")\n        global_log.see(\"end\")\n"""
    text = replace_once(text, old, new, "colored summary GUI tags")

    # Add an explicit V2-7.3 marker next to the existing V2-7.2 marker.
    text = replace_once(
        text,
        'V272_VERSION = "2.7.2"\n',
        'V272_VERSION = "2.7.2"\nV273_VERSION = "2.7.3"\n',
        "V2-7.3 backend marker",
    )

    # Visible version strings only.  Keep V272 helper/function names because
    # V2-7.3 is intentionally incremental and depends on those tested helpers.
    text = text.replace("NERO V2-7.2", "NERO V2-7.3")
    return text


def patch_agent(text: str) -> str:
    if "V2-7.3 TASK SUMMARY" in text:
        raise RuntimeError("Agent already contains V2-7.3 summary patch")

    # Make the execution-layer/grasp-layer distinction explicit to every model.
    policy_addition = '''\nV2-7.3 execution-layer rule:\n- If move_to_pregrasp returns reason=PREGRASP_PHYSICAL_TRACKING_TIMEOUT, the selected grasp geometry has NOT been disproved. Do NOT call plan_grasp again and do NOT try another candidate. Stop the task and report a controller/dynamics tracking timeout.\n- A LATE_SETTLE_RECOVERED warning is recoverable: continue from the successfully reached pregrasp, but preserve the warning for the final task summary.\n'''
    anchor = "\nOperating policy:"
    if anchor not in text:
        raise RuntimeError("Agent patch: Operating policy anchor missing")
    text = text.replace(anchor, policy_addition + anchor, 1)

    # Track the deterministic episode record used by the final summary.
    old = """        tool_count = 0\n        multi_object_guard_nudges = 0\n"""
    new = """        tool_count = 0\n        multi_object_guard_nudges = 0\n\n        # V2-7.3 TASK SUMMARY — deterministic episode bookkeeping.\n        v273_tool_history = []\n        v273_final_text = \"\"\n        v273_exception_text = None\n\n        def _v273_emit_high_level_summary():\n            history = list(v273_tool_history)\n            failed = []\n            warning_items = []\n            verified_objects = set()\n            terminal_verify_count = 0\n\n            for item in history:\n                tool = str(item.get(\"tool\", \"\"))\n                result = item.get(\"result\") or {}\n                if not isinstance(result, dict):\n                    continue\n                ok = bool(result.get(\"ok\", False))\n                if not ok:\n                    pv = result.get(\"physical_verification\") or {}\n                    reason = (\n                        pv.get(\"reason\")\n                        or result.get(\"error\")\n                        or result.get(\"reason\")\n                        or \"unspecified failure\"\n                    )\n                    failed.append((tool, str(reason)))\n                for w in (result.get(\"warnings\") or []):\n                    warning_items.append(f\"{tool}: {w}\")\n                if tool in (\"verify_object_in_slot\", \"verify_object_in_bin\"):\n                    terminal_verify_count += 1\n                    if ok:\n                        obj = (\n                            result.get(\"object_name\")\n                            or (result.get(\"physical_verification\") or {}).get(\"object_name\")\n                            or (result.get(\"physical_verification\") or {}).get(\"object\")\n                        )\n                        if obj:\n                            verified_objects.add(str(obj))\n\n            try:\n                requested_raw = self._task_object_names(task, None) or []\n            except Exception:\n                requested_raw = []\n            requested = [\n                str(x) for x in requested_raw\n                if x and not str(x).startswith(\"<\")\n            ]\n            all_requested_verified = bool(\n                requested and set(requested).issubset(verified_objects)\n            )\n            any_terminal_verified = bool(verified_objects)\n\n            final_lower = str(v273_final_text or \"\").lower()\n            failure_language = any(\n                key in final_lower\n                for key in (\n                    \"unable to\", \"could not complete\", \"cannot complete\",\n                    \"task failed\", \"failed to\", \"not completed\",\n                    \"infeasible\", \"aborted\"\n                )\n            )\n\n            if v273_exception_text:\n                color = \"RED\"\n                status = \"FAILED\"\n            elif all_requested_verified or (not requested and any_terminal_verified):\n                if failed or warning_items:\n                    color = \"YELLOW\"\n                    status = \"SUCCESS WITH WARNINGS\"\n                else:\n                    color = \"GREEN\"\n                    status = \"SUCCESS\"\n            elif failure_language or failed:\n                color = \"RED\"\n                status = \"FAILED\"\n            else:\n                color = \"YELLOW\"\n                status = \"ENDED WITHOUT FULL PHYSICAL VERIFICATION\"\n\n            if failed:\n                last_failure = f\"{failed[-1][0]}: {failed[-1][1]}\"\n            elif v273_exception_text:\n                last_failure = str(v273_exception_text)\n            else:\n                last_failure = \"none\"\n\n            outcome = str(v273_final_text or \"\").strip().replace(\"\\n\", \" \" )\n            if not outcome:\n                outcome = \"No final model text was produced.\"\n            if len(outcome) > 600:\n                outcome = outcome[:597] + \"...\"\n\n            summary = (\n                f\"[TASK SUMMARY {color}]\\n\"\n                f\"STATUS: {status}\\n\"\n                f\"TASK: {task}\\n\"\n                f\"OUTCOME: {outcome}\\n\"\n                f\"TOOLS: {len(history)} total | {len(failed)} failed | \"\n                f\"{len(warning_items)} warnings\\n\"\n                f\"PHYSICAL VERIFY: {len(verified_objects)} object(s) verified | \"\n                f\"requested={requested or 'unspecified'}\\n\"\n                f\"LAST FAILURE: {last_failure}\"\n            )\n            self.emit(summary)\n"""
    text = replace_once(text, old, new, "summary bookkeeping")

    # Preserve final model text for the deterministic summary.
    old = """                    self._last_final_text = text\n                    self.emit(\n                        f\"[LLM TASK COMPLETE] tool_steps={tool_count} | \"\n"""
    new = """                    self._last_final_text = text\n                    v273_final_text = text\n                    self.emit(\n                        f\"[LLM TASK COMPLETE] tool_steps={tool_count} | \"\n"""
    text = replace_once(text, old, new, "capture final text")

    # Store every tool result after it has been normalized by the Agent loop.
    old = """                    outputs.append(\n                        {\n                            \"type\": \"function_call_output\",\n                            \"call_id\": call_id,\n                            \"output\": json.dumps(result, ensure_ascii=False),\n                        }\n                    )\n"""
    new = """                    v273_tool_history.append({\"tool\": name, \"result\": result})\n                    outputs.append(\n                        {\n                            \"type\": \"function_call_output\",\n                            \"call_id\": call_id,\n                            \"output\": json.dumps(result, ensure_ascii=False),\n                        }\n                    )\n"""
    text = replace_once(text, old, new, "record tool result")

    # Summary MUST be printed after TASK END.  This is intentionally inside the
    # finally block so failures/exceptions also produce a red summary.
    old = """        except Exception as exc:\n            self.emit(f\"[LLM TASK FAILED] {exc}\")\n            raise\n        finally:\n            with self._lock:\n                self._running = False\n            self.emit(f\"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s\")\n"""
    new = """        except Exception as exc:\n            v273_exception_text = str(exc)\n            self.emit(f\"[LLM TASK FAILED] {exc}\")\n            raise\n        finally:\n            with self._lock:\n                self._running = False\n            self.emit(f\"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s\")\n            try:\n                _v273_emit_high_level_summary()\n            except Exception as _summary_exc:\n                self.emit(\n                    \"[TASK SUMMARY RED]\\n\"\n                    \"STATUS: FAILED\\n\"\n                    f\"TASK: {task}\\n\"\n                    f\"OUTCOME: Task-summary generation failed: {_summary_exc}\"\n                )\n"""
    text = replace_once(text, old, new, "emit summary after task end")

    text = text.replace("NERO V2-7.2", "NERO V2-7.3")
    return text


def patch_launcher(text: str, out_name: str) -> str:
    text = text.replace("V2-7.2", "V2-7.3")
    text = text.replace(
        "NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation",
        out_name,
    )
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
            raise FileNotFoundError(f"Missing V2-7.2 runtime file: {src/name}")

    core_hash = sha256(src / CORE)
    if core_hash != EXPECTED_V69_SHA256:
        raise RuntimeError(
            "Frozen V69 SHA mismatch. Refusing to patch.\n"
            f"Expected: {EXPECTED_V69_SHA256}\n"
            f"Actual:   {core_hash}"
        )

    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)

    backend = out / BACKEND
    agent = out / AGENT
    backend.write_text(patch_backend(backend.read_text(encoding="utf-8")), encoding="utf-8")
    agent.write_text(patch_agent(agent.read_text(encoding="utf-8")), encoding="utf-8")

    old_launcher = out / "start_nero_v2_7_2.sh"
    new_launcher = out / "start_nero_v2_7_3.sh"
    if old_launcher.is_file():
        new_launcher.write_text(
            patch_launcher(old_launcher.read_text(encoding="utf-8"), out.name),
            encoding="utf-8",
        )
    else:
        new_launcher.write_text(
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            'cd "$(dirname "$0")"\n'
            'exec python3 NERO_V2_6_Unified_Backend.py\n',
            encoding="utf-8",
        )
    new_launcher.chmod(new_launcher.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    # Static validation of the GENERATED runtime, not merely this installer.
    py_compile.compile(str(backend), doraise=True)
    py_compile.compile(str(agent), doraise=True)
    subprocess.run(["bash", "-n", str(new_launcher)], check=True)

    out_core_hash = sha256(out / CORE)
    if out_core_hash != EXPECTED_V69_SHA256:
        raise RuntimeError("V69 changed during V2-7.3 generation; aborting")

    btxt = backend.read_text(encoding="utf-8")
    atxt = agent.read_text(encoding="utf-8")
    required_backend = [
        'V273_VERSION = "2.7.3"',
        "V2-7.3 LATE SETTLE WATCH",
        "PREGRASP_PHYSICAL_TRACKING_TIMEOUT",
        "task_summary_success",
        "task_summary_warning",
        "task_summary_failure",
    ]
    required_agent = [
        "V2-7.3 TASK SUMMARY",
        "PREGRASP_PHYSICAL_TRACKING_TIMEOUT",
        "[TASK SUMMARY {color}]",
        "_v273_emit_high_level_summary",
    ]
    missing = [x for x in required_backend if x not in btxt] + [x for x in required_agent if x not in atxt]
    if missing:
        raise RuntimeError(f"V2-7.3 static marker check failed: {missing}")

    (out / "V2_7_3_NOTES.md").write_text(
        """# NERO V2-7.3 — Late-Settle Verification + Task-End Summary\n\n"
        "Changes:\n"
        "- Frozen V69 remains byte-for-byte unchanged.\n"
        "- Extends wrapper-only passive late-settle verification from 1.8 s to 4.5 s.\n"
        "- A recovered late settle is SUCCESS_WITH_WARNING, not a grasp-candidate failure.\n"
        "- Unrecovered PREGRASP_PHYSICAL_TRACKING_TIMEOUT no longer consumes/switches grasp candidates.\n"
        "- After [LLM TASK END], prints one deterministic high-level summary.\n"
        "- Green = physically verified success with no warnings.\n"
        "- Yellow = success with warnings/recovered failures, or ended without full physical verification.\n"
        "- Red = failure/exception.\n"
        "- Existing V2-7.2 egg labels, hold continuity, bounded wrist reorientation, and Copy Full Log remain intact.\n"
        """,
        encoding="utf-8",
    )

    print("[OK] NERO V2-7.3 runtime created:", out)
    print("[OK] Backend syntax")
    print("[OK] Agent syntax")
    print("[OK] Launcher syntax")
    print("[OK] Frozen V69 SHA256:", out_core_hash)
    print("[OK] 4.5 s late-settle recheck installed")
    print("[OK] Tracking mismatch no longer poisons grasp candidate selection")
    print("[OK] Colored task-end summary installed")
    print("[RUN]", new_launcher)


if __name__ == "__main__":
    main()
