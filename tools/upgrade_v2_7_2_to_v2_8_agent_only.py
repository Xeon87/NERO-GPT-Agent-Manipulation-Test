#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import py_compile
import shutil
import stat
from pathlib import Path

EXPECTED_V69_SHA256 = "3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
BACKEND = "NERO_V2_7_1_Unified_Backend.py"
AGENT = "NERO_V2_7_1_Agent.py"
CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"
LAUNCHER = "start_nero_v2_7_2.sh"
OUT_DIR_NAME = "NERO_V2_8_Agent_Orchestration_Only_From_V272"


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


def patch_agent(text: str) -> str:
    # Keep everything from the validated V2-7.2 Agent except the two
    # orchestration fixes below. No GUI, backend, summary, stop, or motion edits.
    if "approach_planned_grasp" not in text or "multi_object_guard_nudges" not in text:
        raise RuntimeError("Source Agent does not look like validated V2-7.2")

    # Version marker only; behavior changes are limited to the two blocks below.
    text = text.replace('PROJECT_VERSION = "2.6.0"', 'PROJECT_VERSION = "2.8.0"', 1)

    old_state = '''        tool_count = 0\n        multi_object_guard_nudges = 0\n\n        try:\n'''
    new_state = '''        tool_count = 0\n        multi_object_guard_nudges = 0\n        # V2-8 orchestration-only state: requested objects that the deterministic\n        # planner has proven terminally infeasible in the current episode.\n        v28_terminal_objects = set()\n\n        try:\n'''
    text = replace_once(text, old_state, new_state, "terminal object state")

    old_pending = '''                            guard_result = self.tool_runner("observe_scene", {})\n                            guard_scene = guard_result.get("scene") if isinstance(guard_result, dict) else None\n                            pending = self._task_object_names(task, guard_scene or {})\n'''
    new_pending = '''                            guard_result = self.tool_runner("observe_scene", {})\n                            guard_scene = guard_result.get("scene") if isinstance(guard_result, dict) else None\n                            _v28_pending_raw = self._task_object_names(task, guard_scene or {})\n                            pending = [\n                                _name for _name in _v28_pending_raw\n                                if str(_name) not in v28_terminal_objects\n                            ]\n                            if _v28_pending_raw and not pending and v28_terminal_objects:\n                                self.emit(\n                                    "[LLM ORCHESTRATOR TERMINAL] all scene-pending requested objects "\n                                    f"are terminal-blocked: {sorted(v28_terminal_objects)}"\n                                )\n'''
    text = replace_once(text, old_pending, new_pending, "terminal-aware pending filter")

    old_tool_result = '''                            self.emit(\n                                f"[LLM TOOL RESULT #{tool_count}] {name} | "\n                                f"elapsed={time.perf_counter()-t_tool:.2f}s | "\n                                + json.dumps(result, ensure_ascii=False, separators=(",", ":"))\n                            )\n\n                    outputs.append(\n                        {\n                            "type": "function_call_output",\n                            "call_id": call_id,\n                            "output": json.dumps(result, ensure_ascii=False),\n                        }\n                    )\n'''
    new_tool_result = '''                            self.emit(\n                                f"[LLM TOOL RESULT #{tool_count}] {name} | "\n                                f"elapsed={time.perf_counter()-t_tool:.2f}s | "\n                                + json.dumps(result, ensure_ascii=False, separators=(",", ":"))\n                            )\n\n                    # --------------------------------------------------------\n                    # V2-8 Agent orchestration fix #2:\n                    # remember objects that deterministic grasp planning has\n                    # proven terminally infeasible, so the plural completion\n                    # guard does not force the model into an endless retry loop.\n                    # --------------------------------------------------------\n                    if name == "plan_grasp" and isinstance(result, dict) and not bool(result.get("ok", False)):\n                        _v28_obj = str(args.get("object_name", "")) if isinstance(args, dict) else ""\n                        _v28_pv = result.get("physical_verification") or {}\n                        _v28_reason = " | ".join(\n                            str(x) for x in (\n                                result.get("reason"),\n                                _v28_pv.get("reason") if isinstance(_v28_pv, dict) else None,\n                                result.get("error"),\n                            ) if x\n                        )\n                        if _v28_obj and (\n                            "ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE" in _v28_reason\n                            or "REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7" in _v28_reason\n                        ):\n                            v28_terminal_objects.add(_v28_obj)\n                            self.emit(\n                                f"[LLM ORCHESTRATOR TERMINAL] {_v28_obj} terminal-blocked | reason={_v28_reason}"\n                            )\n\n                    # --------------------------------------------------------\n                    # V2-8 Agent orchestration fix #1:\n                    # once the deterministic approach has physically reached\n                    # GRASP_POSE_REACHED, clamp immediately in the SAME\n                    # orchestration cycle. Safety remains entirely inside the\n                    # existing clamp_gripper backend skill.\n                    # --------------------------------------------------------\n                    if (\n                        not dry_run\n                        and len(calls) == 1\n                        and name == "approach_planned_grasp"\n                        and isinstance(result, dict)\n                        and bool(result.get("ok", False))\n                        and str((result.get("state_after") or {}).get("stage", "")) == "GRASP_POSE_REACHED"\n                    ):\n                        if tool_count >= max_tool_steps:\n                            raise RuntimeError(\n                                f"LLM reached max_tool_steps={max_tool_steps} before required auto-clamp"\n                            )\n                        self.emit(\n                            "[LLM ORCHESTRATOR AUTO-CLAMP] verified approach -> "\n                            "clamp_gripper(force_n=5.0) without another LLM round-trip"\n                        )\n                        tool_count += 1\n                        _v28_t_clamp = time.perf_counter()\n                        try:\n                            _v28_clamp = self.tool_runner("clamp_gripper", {"force_n": 5.0})\n                            if not isinstance(_v28_clamp, dict):\n                                _v28_clamp = {"ok": bool(_v28_clamp), "result": _v28_clamp}\n                        except Exception as _v28_exc:\n                            _v28_clamp = {\n                                "ok": False,\n                                "tool": "clamp_gripper",\n                                "error": str(_v28_exc),\n                            }\n                        self.emit(\n                            f"[LLM TOOL RESULT #{tool_count}] clamp_gripper[auto] | "\n                            f"elapsed={time.perf_counter()-_v28_t_clamp:.2f}s | "\n                            + json.dumps(_v28_clamp, ensure_ascii=False, separators=(",", ":"))\n                        )\n\n                        _v28_combined = dict(result)\n                        _v28_combined["orchestration_auto_followup"] = {\n                            "tool": "clamp_gripper",\n                            "arguments": {"force_n": 5.0},\n                            "result": _v28_clamp,\n                        }\n                        _v28_combined["ok"] = bool(\n                            result.get("ok", False) and _v28_clamp.get("ok", False)\n                        )\n                        if isinstance(_v28_clamp.get("state_after"), dict):\n                            _v28_combined["state_after"] = _v28_clamp["state_after"]\n                        if "physical_verification" in _v28_clamp:\n                            _v28_combined["clamp_physical_verification"] = _v28_clamp["physical_verification"]\n                        if _v28_clamp.get("warnings"):\n                            _v28_combined["warnings"] = list(_v28_combined.get("warnings") or []) + list(_v28_clamp.get("warnings") or [])\n                        if "recommended_next_actions" in _v28_clamp:\n                            _v28_combined["recommended_next_actions"] = _v28_clamp["recommended_next_actions"]\n                        result = _v28_combined\n\n                    outputs.append(\n                        {\n                            "type": "function_call_output",\n                            "call_id": call_id,\n                            "output": json.dumps(result, ensure_ascii=False),\n                        }\n                    )\n'''
    text = replace_once(text, old_tool_result, new_tool_result, "two orchestration fixes")

    return text


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("source_dir")
    ap.add_argument("--output-dir", default=OUT_DIR_NAME)
    args = ap.parse_args()

    src = Path(args.source_dir).resolve()
    out = Path(args.output_dir).resolve()
    if out.exists():
        shutil.rmtree(out)
    shutil.copytree(src, out)

    # Remove generated caches from the distributable package.
    for p in out.rglob("__pycache__"):
        shutil.rmtree(p, ignore_errors=True)

    for required in (BACKEND, AGENT, CORE, LAUNCHER):
        if not (out / required).is_file():
            raise RuntimeError(f"Missing V2-7.2 runtime file: {required}")

    # V69 must remain frozen.
    if sha256(out / CORE) != EXPECTED_V69_SHA256:
        raise RuntimeError("Frozen V69 SHA256 mismatch")

    # Save exact V2-7.2 infrastructure hashes before touching Agent.
    baseline_backend_sha = sha256(out / BACKEND)
    baseline_launcher_sha = sha256(out / LAUNCHER)
    baseline_core_sha = sha256(out / CORE)

    agent_path = out / AGENT
    agent_path.write_text(patch_agent(agent_path.read_text(encoding="utf-8")), encoding="utf-8")

    # V2-8 user-facing launcher is an EXACT byte copy of the V2-7.2 launcher.
    # This preserves every startup/environment/process detail while providing a
    # version-appropriate command name.
    launcher_v28 = out / "start_nero_v2_8.sh"
    shutil.copyfile(out / LAUNCHER, launcher_v28)
    launcher_v28.chmod(launcher_v28.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    # Prove infrastructure was not touched.
    if sha256(out / BACKEND) != baseline_backend_sha:
        raise RuntimeError("Backend changed unexpectedly")
    if sha256(out / LAUNCHER) != baseline_launcher_sha:
        raise RuntimeError("V2-7.2 launcher changed unexpectedly")
    if sha256(out / CORE) != baseline_core_sha:
        raise RuntimeError("V69 changed unexpectedly")
    if (out / LAUNCHER).read_bytes() != launcher_v28.read_bytes():
        raise RuntimeError("V2-8 launcher is not byte-identical to V2-7.2 launcher")

    py_compile.compile(str(out / BACKEND), doraise=True)
    py_compile.compile(str(out / AGENT), doraise=True)
    py_compile.compile(str(out / CORE), doraise=True)

    readme = """# NERO V2-8 — Agent Orchestration Only\n\nBaseline: validated NERO V2-7.2 runtime.\n\nOnly two behavioral changes are present, both inside the Agent:\n1. Verified `approach_planned_grasp` -> deterministic immediate `clamp_gripper(force_n=5.0)` in the same orchestration cycle.\n2. Plural-task completion guard excludes objects that deterministic `plan_grasp` has already proven terminally infeasible (`ALL_GRASP_CANDIDATES_EXHAUSTED_OR_INFEASIBLE` / `REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7`).\n\nIntentionally unchanged from V2-7.2:\n- MuJoCo backend and multiprocessing spawn path\n- `run_sim()` / `mujoco.viewer.launch_passive()`\n- Tk GUI\n- RViz / MoveIt / BioIK / TRAC-IK startup\n- V69 motion core\n- grasp geometry, placement, hold continuity, bounded wrist logic\n- user-stop behavior\n- task-summary behavior\n- GUI log styling\n\n`start_nero_v2_8.sh` is byte-for-byte identical to the validated V2-7.2 launcher.\n"""
    (out / "README_V2_8.md").write_text(readme, encoding="utf-8")

    path_check = """NERO V2-8 package contract\n==========================\n./NERO_V2_7_1_Unified_Backend.py   (EXACT V2-7.2 backend)\n./NERO_V2_7_1_Agent.py             (V2-7.2 Agent + 2 orchestration fixes only)\n./NERO_V69_ReachableZone_BioIK_Planner_Core.py  (frozen)\n./start_nero_v2_7_2.sh             (EXACT V2-7.2 launcher)\n./start_nero_v2_8.sh               (byte-identical copy of V2-7.2 launcher)\n./README_V2_8.md\n./SHA256SUMS.txt\n"""
    (out / "PATH_CHECK.txt").write_text(path_check, encoding="utf-8")

    hashes = []
    for name in (BACKEND, AGENT, CORE, LAUNCHER, "start_nero_v2_8.sh"):
        hashes.append(f"{sha256(out / name)}  {name}")
    (out / "SHA256SUMS.txt").write_text("\n".join(hashes) + "\n", encoding="utf-8")

    print(f"[OK] V2-8 package created: {out}")
    print(f"[OK] exact V2-7.2 backend SHA256: {baseline_backend_sha}")
    print(f"[OK] exact V2-7.2 launcher SHA256: {baseline_launcher_sha}")
    print(f"[OK] frozen V69 SHA256: {baseline_core_sha}")
    print("[OK] ONLY Agent behavior changed: auto-clamp + terminal-aware plural completion")


if __name__ == "__main__":
    main()
