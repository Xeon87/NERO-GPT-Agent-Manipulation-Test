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
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one anchor, found {count}")
    return text.replace(old, new, 1)


def patch_backend(text: str) -> str:
    if 'V272_VERSION = "2.7.2"' not in text:
        raise RuntimeError("Source backend is not the validated V2-7.2 runtime")
    if "V2-9 LOG STYLE" in text:
        raise RuntimeError("Backend already contains V2-9 log styling")

    old = '''    def append_gui_log(text):
        global_log.insert("end", str(text).rstrip() + "\\n")
        global_log.see("end")
'''
    new = '''    # V2-9 LOG STYLE — presentation only. No robot/control/orchestration changes.
    global_log.tag_configure(
        "llm_message_bold", font=("Arial", 10, "bold")
    )
    global_log.tag_configure(
        "task_summary_success", foreground="#138A36", font=("Arial", 10, "bold")
    )
    global_log.tag_configure(
        "task_summary_warning", foreground="#A66B00", font=("Arial", 10, "bold")
    )
    global_log.tag_configure(
        "task_summary_failure", foreground="#C62828", font=("Arial", 10, "bold")
    )

    def append_gui_log(text):
        _msg = str(text).rstrip()
        _tag = None
        if _msg.startswith("[TASK SUMMARY GREEN]"):
            _tag = "task_summary_success"
        elif _msg.startswith("[TASK SUMMARY YELLOW]"):
            _tag = "task_summary_warning"
        elif _msg.startswith("[TASK SUMMARY RED]"):
            _tag = "task_summary_failure"
        elif _msg.startswith("[LLM"):
            _tag = "llm_message_bold"

        if _tag:
            global_log.insert("end", _msg + "\\n", _tag)
        else:
            global_log.insert("end", _msg + "\\n")
        global_log.see("end")
'''
    text = replace_once(text, old, new, "append_gui_log styling")

    # V2-9 version marker/display only; keep all V2-7.2 implementation names.
    text = replace_once(
        text,
        'V272_VERSION = "2.7.2"\n',
        'V272_VERSION = "2.7.2"\nV29_VERSION = "2.9"\n',
        "V2-9 version marker",
    )
    text = text.replace("NERO V2-7.2", "NERO V2-9")
    return text


def patch_agent(text: str) -> str:
    if "V2-9 TASK SUMMARY" in text:
        raise RuntimeError("Agent already contains V2-9 summary patch")

    old = '''        tool_count = 0
        multi_object_guard_nudges = 0
'''
    new = '''        tool_count = 0
        multi_object_guard_nudges = 0

        # V2-9 TASK SUMMARY — deterministic bookkeeping only.
        # This does not alter tool selection, retries, safety, or orchestration.
        v29_tool_history = []
        v29_final_text = ""
        v29_exception_text = None

        def _v29_emit_high_level_summary():
            history = list(v29_tool_history)
            failed = []
            warning_items = []
            verified_objects = set()

            for item in history:
                tool = str(item.get("tool", ""))
                result = item.get("result") or {}
                if not isinstance(result, dict):
                    continue

                ok = bool(result.get("ok", False))
                pv = result.get("physical_verification") or {}
                if not isinstance(pv, dict):
                    pv = {}

                if not ok:
                    reason = (
                        pv.get("reason")
                        or result.get("error")
                        or result.get("reason")
                        or "unspecified failure"
                    )
                    failed.append((tool, str(reason)))

                warnings = result.get("warnings") or []
                if isinstance(warnings, str):
                    warnings = [warnings]
                for warning in warnings:
                    warning_items.append(f"{tool}: {warning}")

                if tool in ("verify_object_in_slot", "verify_object_in_bin") and ok:
                    obj = (
                        result.get("object_name")
                        or pv.get("object_name")
                        or pv.get("object")
                    )
                    if obj:
                        verified_objects.add(str(obj))

            try:
                requested_raw = self._task_object_names(task, None) or []
            except Exception:
                requested_raw = []
            requested = [
                str(x) for x in requested_raw
                if x and not str(x).startswith("<")
            ]

            all_requested_verified = bool(
                requested and set(requested).issubset(verified_objects)
            )
            any_terminal_verified = bool(verified_objects)

            final_lower = str(v29_final_text or "").lower()
            failure_language = any(
                key in final_lower
                for key in (
                    "unable to", "could not complete", "cannot complete",
                    "task failed", "failed to", "not completed",
                    "infeasible", "aborted"
                )
            )

            if v29_exception_text:
                color = "RED"
                status = "FAILED"
            elif all_requested_verified or (not requested and any_terminal_verified):
                if failed or warning_items:
                    color = "YELLOW"
                    status = "SUCCESS WITH WARNINGS"
                else:
                    color = "GREEN"
                    status = "SUCCESS"
            elif failure_language or failed:
                color = "RED"
                status = "FAILED"
            else:
                color = "YELLOW"
                status = "ENDED WITHOUT FULL PHYSICAL VERIFICATION"

            if failed:
                last_failure = f"{failed[-1][0]}: {failed[-1][1]}"
            elif v29_exception_text:
                last_failure = str(v29_exception_text)
            else:
                last_failure = "none"

            outcome = str(v29_final_text or "").strip().replace("\\n", " ")
            if not outcome:
                outcome = "No final model text was produced."
            if len(outcome) > 600:
                outcome = outcome[:597] + "..."

            summary = (
                f"[TASK SUMMARY {color}]\\n"
                f"STATUS: {status}\\n"
                f"TASK: {task}\\n"
                f"OUTCOME: {outcome}\\n"
                f"TOOLS: {len(history)} total | {len(failed)} failed | "
                f"{len(warning_items)} warnings\\n"
                f"PHYSICAL VERIFY: {len(verified_objects)} object(s) verified | "
                f"requested={requested or 'unspecified'}\\n"
                f"LAST FAILURE: {last_failure}"
            )
            self.emit(summary)
'''
    text = replace_once(text, old, new, "summary bookkeeping")

    old = '''                    self._last_final_text = text
                    self.emit(
                        f"[LLM TASK COMPLETE] tool_steps={tool_count} | "
'''
    new = '''                    self._last_final_text = text
                    v29_final_text = text
                    self.emit(
                        f"[LLM TASK COMPLETE] tool_steps={tool_count} | "
'''
    text = replace_once(text, old, new, "capture final text")

    old = '''                    outputs.append(
                        {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps(result, ensure_ascii=False),
                        }
                    )
'''
    new = '''                    v29_tool_history.append({"tool": name, "result": result})
                    outputs.append(
                        {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps(result, ensure_ascii=False),
                        }
                    )
'''
    text = replace_once(text, old, new, "record tool result")

    old = '''        except Exception as exc:
            self.emit(f"[LLM TASK FAILED] {exc}")
            raise
        finally:
            with self._lock:
                self._running = False
            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")
'''
    new = '''        except Exception as exc:
            v29_exception_text = str(exc)
            self.emit(f"[LLM TASK FAILED] {exc}")
            raise
        finally:
            with self._lock:
                self._running = False
            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")
            try:
                _v29_emit_high_level_summary()
            except Exception as _summary_exc:
                self.emit(
                    "[TASK SUMMARY RED]\\n"
                    "STATUS: FAILED\\n"
                    f"TASK: {task}\\n"
                    f"OUTCOME: Task-summary generation failed: {_summary_exc}"
                )
'''
    text = replace_once(text, old, new, "emit summary after task end")

    # Metadata/display only.
    if 'PROJECT_VERSION = "2.7.1"' in text:
        text = text.replace('PROJECT_VERSION = "2.7.1"', 'PROJECT_VERSION = "2.9.0"', 1)
    elif 'PROJECT_VERSION = "2.7.0"' in text:
        text = text.replace('PROJECT_VERSION = "2.7.0"', 'PROJECT_VERSION = "2.9.0"', 1)
    text = text.replace("NERO V2-7.2", "NERO V2-9")
    return text


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("runtime_dir", type=Path)
    args = ap.parse_args()

    runtime = args.runtime_dir.expanduser().resolve()
    backend = runtime / BACKEND
    agent = runtime / AGENT
    core = runtime / CORE

    for path in (backend, agent, core, runtime / "start_nero_v2_7_2.sh"):
        if not path.is_file():
            raise FileNotFoundError(f"Missing validated V2-7.2 runtime file: {path}")

    if sha256(core) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 SHA256 mismatch; refusing V2-9 patch")

    baseline_backend = backend.read_bytes()
    baseline_agent = agent.read_bytes()
    baseline_core = core.read_bytes()
    baseline_launcher = (runtime / "start_nero_v2_7_2.sh").read_bytes()

    backend.write_text(patch_backend(backend.read_text(encoding="utf-8")), encoding="utf-8")
    agent.write_text(patch_agent(agent.read_text(encoding="utf-8")), encoding="utf-8")

    py_compile.compile(str(backend), doraise=True)
    py_compile.compile(str(agent), doraise=True)

    if core.read_bytes() != baseline_core:
        raise RuntimeError("V69 changed unexpectedly")
    if (runtime / "start_nero_v2_7_2.sh").read_bytes() != baseline_launcher:
        raise RuntimeError("V2-7.2 runtime launcher changed unexpectedly")

    # Keep deterministic audit copies of the exact pre-patch files for diffing.
    (runtime / "V2_9_BASELINE_BACKEND_SHA256.txt").write_text(
        hashlib.sha256(baseline_backend).hexdigest() + "\n", encoding="utf-8"
    )
    (runtime / "V2_9_BASELINE_AGENT_SHA256.txt").write_text(
        hashlib.sha256(baseline_agent).hexdigest() + "\n", encoding="utf-8"
    )

    print("[OK] NERO V2-9 presentation/summary patch applied IN PLACE to exact V2-7.2 runtime")
    print("[OK] V69 unchanged:", EXPECTED_V69_SHA256)
    print("[OK] start_nero_v2_7_2.sh unchanged byte-for-byte")
    print("[OK] Backend delta: GUI log styling + V2-9 display marker only")
    print("[OK] Agent delta: task-end summary bookkeeping/emission + version metadata only")


if __name__ == "__main__":
    main()
