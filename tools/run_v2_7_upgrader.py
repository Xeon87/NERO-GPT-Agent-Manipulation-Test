#!/usr/bin/env python3
"""Run the V2-7 local upgrade payload from this experimental branch.

This wrapper applies packaging/runtime compatibility fixes so the upgrader:
- accepts the actual V2-6 Egg Task Semantic Integration Agent layout;
- enforces the successful V2-7 approach -> clamp transition deterministically;
- emits a local high-level episode summary even when the user stops the Agent.

V69 is not modified.
"""
from pathlib import Path

payload = Path(__file__).with_name("upgrade_v2_6_egg_to_v2_7.py")
text = payload.read_text(encoding="utf-8")

# Repair a packaging-only quote typo in the payload's generated README text.
text = text.replace(
    '        """# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    '        "# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    1,
)

# Compatibility hotfix for the real V2-6 Egg Semantic Agent. Earlier public
# Agents had literal headings that the Egg Semantic Integration Agent can omit.
old_guide_block = '''    # Insert the policy before the existing state-machine/contract section.\n    if "\\nState machine:\\n" in text:\n        text = text.replace("\\nState machine:\\n", "\\n" + AGENT_V27_GUIDE + "\\nState machine:\\n", 1)\n    elif "\\nContract rules:\\n" in text:\n        text = text.replace("\\nContract rules:\\n", "\\n" + AGENT_V27_GUIDE + "\\nContract rules:\\n", 1)\n    else:\n        raise RuntimeError("Could not find Agent guide insertion point")\n'''
new_guide_block = '''    # Insert the policy before the existing state-machine/contract section.\n    if "\\nState machine:\\n" in text:\n        text = text.replace("\\nState machine:\\n", "\\n" + AGENT_V27_GUIDE + "\\nState machine:\\n", 1)\n    elif "\\nContract rules:\\n" in text:\n        text = text.replace("\\nContract rules:\\n", "\\n" + AGENT_V27_GUIDE + "\\nContract rules:\\n", 1)\n    elif "Operating policy:\\n" in text:\n        text = text.replace("Operating policy:\\n", AGENT_V27_GUIDE + "\\nOperating policy:\\n", 1)\n    elif "TOOLS = [\\n" in text:\n        text = text.replace("TOOLS = [\\n", AGENT_V27_GUIDE + "\\nTOOLS = [\\n", 1)\n    else:\n        raise RuntimeError("Could not find a stable Agent guide insertion point")\n'''
if old_guide_block not in text:
    raise RuntimeError("Packaging hotfix could not locate the Agent guide patch block")
text = text.replace(old_guide_block, new_guide_block, 1)

# The Egg Semantic Agent may call this section "Operating policy:" or may have
# already folded those instructions into its main prompt. Missing prose is not
# fatal because the V2-7 guide above already contains the mandatory sequence.
old_policy_block = '''    policy_marker = "Operating policy:\\n"\n    extra = (\n        "Operating policy:\\n"\n        "0. V2-7 EGG PICKUP OVERRIDE: for egg_1..egg_4 in the basket, after the mandatory observe_scene and aperture preparation, use plan_grasp -> move_to_pregrasp -> approach_planned_grasp before clamp_gripper. Do not repeatedly use legacy move_above/approach for egg pickup.\\n"\n    )\n    text = replace_once(text, policy_marker, extra, "Agent V2-7 operating policy")\n'''
new_policy_block = '''    policy_marker = "Operating policy:\\n"\n    extra = (\n        "Operating policy:\\n"\n        "0. V2-7 EGG PICKUP OVERRIDE: for egg_1..egg_4 in the basket, after the mandatory observe_scene and aperture preparation, use plan_grasp -> move_to_pregrasp -> approach_planned_grasp before clamp_gripper. Once approach_planned_grasp returns ok=true, the next action is clamp_gripper; do not re-plan, re-align, or approach again.\\n"\n    )\n    if policy_marker in text:\n        text = text.replace(policy_marker, extra, 1)\n'''
if old_policy_block in text:
    text = text.replace(old_policy_block, new_policy_block, 1)

# Inject a payload helper that patches the generated Agent runtime loop. This is
# deliberately orchestration-only: the V69 motion core and backend geometry are
# untouched.
runtime_guard_helper = r'''
def _patch_agent_runtime_guards(text: str) -> str:
    """Add V2-7 deterministic transition and abort-summary guards to run_task."""
    if "V27_POST_APPROACH_CLAMP_LOCK" in text:
        return text

    run_start = text.find("    def run_task(")
    if run_start < 0:
        raise RuntimeError("Could not find Agent run_task() for V2-7 runtime guards")
    prefix = text[:run_start]
    body = text[run_start:]

    init_marker = "        tool_count = 0\n"
    if init_marker not in body:
        raise RuntimeError("Could not find tool_count initialization for V2-7 runtime guards")
    body = body.replace(
        init_marker,
        init_marker
        + "        # V27_POST_APPROACH_CLAMP_LOCK: successful descent is a committed state.\n"
        + "        v27_required_next_tool = None\n"
        + "        v27_transition_guard_nudges = 0\n"
        + "        v27_episode_status = 'RUNNING'\n"
        + "        v27_abort_reason = ''\n"
        + "        v27_episode_history = []\n",
        1,
    )

    # If the model tries to finish immediately after a successful approach,
    # refuse premature completion and explicitly request the mandatory clamp.
    no_calls_marker = "                if not calls:\n"
    if no_calls_marker not in body:
        raise RuntimeError("Could not find no-calls branch for V2-7 clamp lock")
    no_calls_guard = '''                if not calls:\n                    if v27_required_next_tool == "clamp_gripper":\n                        v27_transition_guard_nudges += 1\n                        if v27_transition_guard_nudges > 3:\n                            raise RuntimeError("V2-7 transition guard exhausted while waiting for clamp_gripper")\n                        self.emit(\n                            f"[V2-7 TRANSITION GUARD] approach succeeded; final text is premature. "\n                            f"Requesting mandatory clamp_gripper ({v27_transition_guard_nudges}/3)."\n                        )\n                        payload = {\n                            "model": str(model),\n                            "instructions": SYSTEM_INSTRUCTIONS,\n                            "previous_response_id": self._last_response_id,\n                            "input": (\n                                "V2-7 deterministic state-machine guard: the latest "\n                                "approach_planned_grasp succeeded and the TCP is already at "\n                                "GRASP_POSE_REACHED. The next action MUST be clamp_gripper. "\n                                "Do not plan_grasp, move_to_pregrasp, approach again, observe, "\n                                "or give a final summary before clamp_gripper."\n                            ),\n                            "tools": TOOLS,\n                            "tool_choice": "auto",\n                            "parallel_tool_calls": False,\n                            "reasoning": {"effort": str(reasoning_effort)},\n                            "max_output_tokens": 1200,\n                            "store": True,\n                        }\n                        continue\n'''
    body = body.replace(no_calls_marker, no_calls_guard, 1)

    # Reject any tool other than clamp_gripper after a successful planned
    # approach. No robot motion is executed for the rejected call.
    raw_args_marker = '                    raw_args = call.get("arguments", "{}") or "{}"\n'
    if raw_args_marker not in body:
        raise RuntimeError("Could not find tool-call argument block for V2-7 clamp lock")
    illegal_transition_guard = '''                    raw_args = call.get("arguments", "{}") or "{}"\n                    if v27_required_next_tool == "clamp_gripper" and name != "clamp_gripper":\n                        v27_transition_guard_nudges += 1\n                        result = {\n                            "ok": False,\n                            "tool": name,\n                            "reason": "ILLEGAL_TRANSITION_AFTER_SUCCESSFUL_APPROACH",\n                            "state_after": "GRASP_POSE_REACHED",\n                            "required_next_action": "clamp_gripper",\n                            "warning": (\n                                "The previous approach_planned_grasp already succeeded. "\n                                "Re-planning/re-aligning/re-approaching is forbidden until clamp_gripper is attempted."\n                            ),\n                        }\n                        self.emit(\n                            f"[V2-7 TRANSITION BLOCK] rejected {name}; "\n                            f"mandatory next tool=clamp_gripper | nudge={v27_transition_guard_nudges}/3"\n                        )\n                        v27_episode_history.append({\n                            "tool": name, "ok": False,\n                            "state_after": "GRASP_POSE_REACHED",\n                            "reason": "ILLEGAL_TRANSITION_AFTER_SUCCESSFUL_APPROACH",\n                        })\n                        outputs.append({\n                            "type": "function_call_output",\n                            "call_id": call_id,\n                            "output": json.dumps(result, ensure_ascii=False),\n                        })\n                        if v27_transition_guard_nudges > 3:\n                            raise RuntimeError("V2-7 transition guard exhausted after repeated illegal post-approach actions")\n                        continue\n'''
    body = body.replace(raw_args_marker, illegal_transition_guard, 1)

    # Record each actual tool result and arm/disarm the post-approach clamp lock.
    output_marker = "\n                    outputs.append(\n"
    if output_marker not in body:
        raise RuntimeError("Could not find function-call output block for V2-7 runtime guards")
    post_tool_guard = '''\n                    if isinstance(result, dict):\n                        v27_reason = result.get("reason")\n                        if not v27_reason:\n                            pv = result.get("physical_verification") or {}\n                            if isinstance(pv, dict):\n                                v27_reason = pv.get("reason")\n                        v27_episode_history.append({\n                            "tool": name,\n                            "ok": bool(result.get("ok")),\n                            "state_after": result.get("state_after"),\n                            "reason": v27_reason,\n                        })\n                        if name == "approach_planned_grasp" and bool(result.get("ok")):\n                            v27_required_next_tool = "clamp_gripper"\n                            v27_transition_guard_nudges = 0\n                            self.emit(\n                                "[V2-7 TRANSITION LOCK] approach_planned_grasp succeeded; "\n                                "next physical action is locked to clamp_gripper"\n                            )\n                        elif name == "clamp_gripper" and v27_required_next_tool == "clamp_gripper":\n                            # Whether clamp succeeds or fails, it has now been attempted.\n                            # Subsequent recovery follows the clamp result contract.\n                            v27_required_next_tool = None\n                            v27_transition_guard_nudges = 0\n\n                    outputs.append(\n'''
    body = body.replace(output_marker, post_tool_guard, 1)

    # Mark natural completion before returning.
    complete_marker = "                    return text\n"
    if complete_marker not in body:
        raise RuntimeError("Could not find normal task return for V2-7 summary guard")
    body = body.replace(
        complete_marker,
        "                    v27_episode_status = 'COMPLETED'\n"
        "                    return text\n",
        1,
    )

    # Replace the terminal exception/finally block with a version that always
    # emits a deterministic local summary, including STOP/KeyboardInterrupt.
    old_tail = '''        except Exception as exc:\n            self.emit(f"[LLM TASK FAILED] {exc}")\n            raise\n        finally:\n            with self._lock:\n                self._running = False\n            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")\n'''
    if old_tail not in body:
        raise RuntimeError("Could not find Agent exception/finally tail for V2-7 abort summary")
    new_tail = '''        except KeyboardInterrupt:\n            v27_episode_status = "INTERRUPTED_BY_KEYBOARD"\n            v27_abort_reason = "KeyboardInterrupt"\n            self.emit("[LLM TASK INTERRUPTED] KeyboardInterrupt")\n            raise\n        except Exception as exc:\n            v27_abort_reason = str(exc)\n            if self._stop_event.is_set() or "stopped by user" in str(exc).lower():\n                v27_episode_status = "STOPPED_BY_USER"\n                self.emit(f"[LLM TASK STOPPED] {exc}")\n            else:\n                v27_episode_status = "FAILED"\n                self.emit(f"[LLM TASK FAILED] {exc}")\n            raise\n        finally:\n            last = v27_episode_history[-1] if v27_episode_history else {}\n            path = " -> ".join(str(x.get("tool")) for x in v27_episode_history[-12:]) or "no_tool_executed"\n            summary_bits = [\n                f"status={v27_episode_status}",\n                f"tool_steps={tool_count}",\n                f"last_tool={last.get('tool')}",\n                f"last_ok={last.get('ok')}",\n                f"last_state={last.get('state_after')}",\n                f"required_next_tool={v27_required_next_tool}",\n                f"elapsed={time.perf_counter()-t_task:.2f}s",\n            ]\n            if last.get("reason"):\n                summary_bits.append(f"last_reason={last.get('reason')}")\n            if v27_abort_reason:\n                summary_bits.append(f"abort_reason={v27_abort_reason}")\n            self.emit("[HIGH-LEVEL SUMMARY] " + " | ".join(summary_bits))\n            self.emit(f"[HIGH-LEVEL PATH] {path}")\n            with self._lock:\n                self._running = False\n            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")\n'''
    body = body.replace(old_tail, new_tail, 1)
    return prefix + body
'''

agent_fn_marker = "\ndef patch_agent(text: str) -> str:\n"
if agent_fn_marker not in text:
    raise RuntimeError("Packaging hotfix could not locate patch_agent()")
text = text.replace(agent_fn_marker, "\n" + runtime_guard_helper + agent_fn_marker, 1)

# Ensure patch_agent() applies the runtime guard after its normal V2-7 prompt/
# tool modifications. Use function boundaries rather than a fragile prose anchor.
agent_start = text.find("\ndef patch_agent(text: str) -> str:\n")
launcher_start = text.find("\ndef patch_launcher(text: str) -> str:\n", agent_start)
if agent_start < 0 or launcher_start < 0:
    raise RuntimeError("Packaging hotfix could not isolate patch_agent()")
agent_block = text[agent_start:launcher_start]
if "text = _patch_agent_runtime_guards(text)" not in agent_block:
    last_return = agent_block.rfind("    return text\n")
    if last_return < 0:
        raise RuntimeError("Packaging hotfix could not locate patch_agent() return")
    agent_block = (
        agent_block[:last_return]
        + "    text = _patch_agent_runtime_guards(text)\n"
        + agent_block[last_return:]
    )
    text = text[:agent_start] + agent_block + text[launcher_start:]

code = compile(text, str(payload), "exec")
ns = {
    "__name__": "__main__",
    "__file__": str(payload),
    "__package__": None,
}
exec(code, ns, ns)
