#!/usr/bin/env python3
"""NERO V2-6 public-release LLM task planner.

This module is deliberately *high level*.  The language model never sends joint
angles, CAN frames, or raw ROS commands.  Instead it chooses from a small set of
robot skills whose preconditions and physical postconditions are enforced by the
backend.  That separation is the main architectural idea of NERO:

    natural-language task
        -> LLM planner
        -> skill contract
        -> deterministic motion / gripper skill
        -> physical verification
        -> updated state / recovery decision

The current transport is the OpenAI Responses API with function calling.  The
model ID is configurable at runtime; the manipulation stack is not tied to one
specific model such as GPT-5.6 Luna.  Provider abstraction is intentionally left
as future work so this public release stays close to the version validated in
MuJoCo.

IMPORTANT
---------
- This file does not directly control real hardware.
- Safety decisions from the deterministic backend remain authoritative.
- Hand-crafted manipulation priors are engineering heuristics, not learned laws.
"""

import json
import os
import threading
import time
import urllib.error
import urllib.request


PROJECT_VERSION = "2.6.0"
PUBLIC_RELEASE_TAG = "v0.1.0"

# The public release uses model-neutral environment-variable names.  The older
# NERO_ASTRA_* names are still accepted so existing local launch setups keep
# working without edits.  The GUI model field is editable and can also be
# populated from GET /v1/models.
DEFAULT_MODEL = os.environ.get(
    "NERO_LLM_MODEL",
    os.environ.get("NERO_ASTRA_MODEL", "gpt-5.6-luna"),
)
DEFAULT_API_BASE = os.environ.get(
    "NERO_LLM_API_BASE",
    os.environ.get("NERO_ASTRA_API_BASE", "https://api.openai.com/v1"),
).rstrip("/")

SKILL_CONTRACT_GUIDE = """\
NERO V2-6 MULTI-OBJECT FAILURE-AWARE SKILL CONTRACTS / PHYSICAL SEMANTICS

The robot tools are deterministic skills with explicit physical consequences. You MUST judge each tool using its contract and the observed continuous scene state, not by guessing what should have happened.

Core principle:
- The gripper does NOT have a simple OPEN/CLOSED semantic state for planning.
- observe_scene reports the continuous actual gripper opening in millimeters and approximate object grasp width.
- Each object has a hand-crafted manipulation prior: a recommended robust gripper-opening range.
- The prior is shape-aware. Spheres require more pre-grasp clearance than their bare diameter because rubber-pad/stud collision envelopes and lateral tracking error can otherwise poke/roll the ball during descent.
- Before approaching a new object, compare actual_opening_mm against that object's recommended range and preferred target.
- If the opening is too narrow OR unnecessarily wide, use set_gripper_opening(target_opening_mm), then observe_scene again before moving toward the object.
- If approach reports PREGRASP_CONTACT_POKE_DETECTED, the backend increases that object's session-local aperture prior. Follow retry_recommended_opening_mm / recommended_next_actions rather than repeating the old aperture.
- The manipulation prior is a robustness recommendation, not a physical law; exact values are intentionally logged for future tuning/learning.
- A collision-check refusal BEFORE execution is also a recoverable pre-grasp failure. If approach returns PREGRASP_PATH_BLOCKED_REQUIRES_ADAPTATION, do not terminate immediately: use the backend-proposed wider aperture / raised grasp target, re-observe, and retry exactly once.
- For plural sorting tasks, scene location is authoritative. Objects already verified inside a matching-color bin are complete; continue with remaining requested objects still in object_zone.

State machine:
OBSERVED -> HOVER_REACHED -> GRASP_POSE_REACHED -> GRASP_ESTABLISHED -> LIFTED -> BIN_HOVER_REACHED -> RELEASED -> PLACEMENT_VERIFIED
A pre-grasp safety trip temporarily enters SAFETY_STOPPED; from there the Agent must observe and may perform one explicit reasoned recovery/retry.

Contract rules:
- observe_scene: reads state only. It does not move the robot or objects.
- set_gripper_opening(target): changes only gripper aperture in position mode before a grasp. It must not be used to release a held object.
- move_above(object): moves only the robot TCP to ~120 mm above the object. The object is EXPECTED TO REMAIN STATIONARY. No grasp occurs yet.
- approach(object): lowers the TCP to the grasp pose. This is a PRE-GRASP NON-CONTACT descent: the object should remain approximately stationary. >4 mm object motion is treated as premature contact/poke and triggers a wider-aperture retry prior.
- recover_from_safety(object, strategy, diagnosis): EXPLICIT recovery for a latched PRE-GRASP safety stop. It is not automatic. First inspect the failure and observe_scene, state a concise diagnosis, then choose a recovery strategy. The tool clears the V69 latch, retreats upward away from the object, and consumes the ONE retry allowed for that object.
- clamp_gripper(force): closes the fingers and establishes provisional contact/hold. The object MAY REMAIN STATIONARY while being clamped.
- lift(height): raises the TCP and the grasped object MUST follow upward. This validates the grasp physically.
- move_above_bin(bin): moves the TCP and the held object together above the bin. The object MUST remain with the gripper during transport.
- open_gripper(): release-only skill used above the bin. It opens/releases the held object; it is NOT the pre-grasp aperture tool.
- verify_object_in_bin(object, bin): checks the final MuJoCo object position. This decides whether placement succeeded.

Every physical tool returns a normalized V2-6 result containing: ok, state_before, state_after, contract, physical_verification, warnings, and recommended_next_actions. Treat normalized ok/state as authoritative for task progression. A legacy V69 diagnostic warning is NOT by itself a task failure if V2-6's deterministic physical postcondition is satisfied. Never treat an explicitly documented expected non-effect as evidence of failure.
"""

SYSTEM_INSTRUCTIONS = f"""You are NERO V2-6, the high-level task planner for a MuJoCo simulated 7-DOF NERO robot arm.
You control the robot ONLY through the provided function tools. Never invent joint angles, ROS commands, shell commands, or direct actuator commands.

{SKILL_CONTRACT_GUIDE}

Operating policy:
1. The first action for any physical task MUST be observe_scene.
2. Use exact object names: red_cube, blue_cube, red_ball, blue_ball.
3. Use exact bin names: red_bin, blue_bin.
4. BEFORE move_above on a new object, inspect:
   - scene.gripper.actual_opening_mm
   - scene.objects[object].grasp_geometry.effective_width_mm
   - scene.objects[object].grasp_geometry.recommended_opening_range_mm
   - scene.objects[object].grasp_geometry.recommended_target_opening_mm
   If current opening is outside the recommended range, call set_gripper_opening using the recommended target, then call observe_scene again to verify the actual opening.
5. Never use open_gripper as the pre-grasp aperture tool. open_gripper is release-only after BIN_HOVER_REACHED.
6. For a normal pick/place after aperture preparation, follow: move_above -> approach -> clamp_gripper -> lift -> move_above_bin -> open_gripper -> verify_object_in_bin.
7. After every tool, read state_after, physical_verification, warnings, and recommended_next_actions before choosing the next tool.
8. Do NOT expect object motion from move_above or approach. In V2-6, >4 mm motion during approach is an anti-poke failure and must be recovered before clamp. Object motion becomes required at lift.
9. Match colors unless the user explicitly requests otherwise: red objects to red_bin; blue objects to blue_bin.
10. After releasing an object, verify placement before starting the next object.
11. If normalized ok=false, do not blindly repeat the same physical action. Observe the scene and reason from the contract, failure reason, object motion, gripper-opening change, safety context and geometry.
12. NERO V2-6 FAILURE-AWARE SAFETY POLICY: if approach returns reason=PREGRASP_SAFETY_STOP_REQUIRES_REFLECTION and safety_reflection_context.retry_allowed=true, the safety stop is NOT the end of the task. You MUST perform exactly one reasoned recovery attempt: (a) call observe_scene while the V69 latch is held; (b) inspect the failure evidence and identify the most likely cause (insufficient aperture/collision envelope, lateral alignment/target-pose error, target motion, or uncertain pre-grasp contact); (c) state a concise diagnosis; (d) call recover_from_safety with that diagnosis and a strategy that materially changes the next attempt; (e) use the returned wider aperture when recommended; (f) call observe_scene AGAIN after recovery so the retry uses fresh object pose/geometry and fresh gripper state; (g) rebuild the approach from the fresh observation and retry once. NEVER blindly reuse the failed target XYZ/path.
13. Exactly ONE safety recovery/retry is allowed per object. If the retry also safety-stops, retry_allowed=false, recovery itself fails, an object is already held, or fresh perception makes the task unsafe, STOP and explain. Never loop and never attempt a third approach.
14. V69 safety trips remain authoritative. Never continue arm motion while the latch is held and never pretend a safety trip was success. recover_from_safety is an explicit planner recovery equivalent to STOP+RECOVER, not a safety override.
15. PLANNING-BLOCK RECOVERY: if approach returns reason=PREGRASP_PATH_BLOCKED_REQUIRES_ADAPTATION and planning_retry_allowed=true, this is NOT a physical collision and no safety unlatch is needed. Follow recommended_next_actions: use the proposed wider aperture, call observe_scene, and retry approach once. The backend also raises the grasp target slightly for that retry. If the second planning attempt is blocked, stop that object and report it; do not loop.
16. MULTI-OBJECT TASKS: for requests such as "pick the objects one by one" or "sort the objects", build the work list from the latest scene. Process every requested object whose location is object_zone, one complete pick/place cycle at a time. Skip objects already in their correct bin. After each verify_object_in_bin, call observe_scene and continue until no requested objects remain in object_zone. A recoverable failure on one step must not be mistaken for completion of the entire multi-object task.
17. Never call multiple physical-motion tools in parallel. Finish each tool and inspect its normalized result before the next action.
18. FRESH EPISODE INVARIANT: a newly confirmed RUN TASK with no held object begins from logical IDLE, regardless of the previous task. Never infer HOVER_REACHED merely because a previous task reached hover before ZERO ALL or before a new RUN TASK. After the mandatory first observe_scene, follow the returned current state only.
19. Keep final task-completion messages concise and factual.
"""

TOOLS = [
    {
        "type": "function",
        "name": "observe_scene",
        "description": "SKILL CONTRACT — Read exact MuJoCo TCP, object pose/approximate dimensions, per-object grasp geometry/manipulation-prior opening range, continuous actual gripper opening, contacts, hold and safety. Observation only; nothing moves. ALWAYS use first. Before a grasp, compare actual_opening_mm with the selected object's recommended_opening_range_mm.",
        "parameters": {"type": "object", "properties": {}, "required": [], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "recover_from_safety",
        "description": "REFLECTIVE SAFETY RECOVERY — Use ONLY after a PRE-GRASP safety stop has produced state=SAFETY_STOPPED and a matching safety_reflection_context. First observe_scene and reason about likely cause. Then explicitly recover the V69 latch, retreat vertically away from the object, and consume the ONE retry allowed for that object. strategy=widen_aperture_and_retry_once adds a wider manipulation-prior margin before retry. If the one retry has already been used, do not call again.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "enum": ["red_cube", "blue_cube", "red_ball", "blue_ball"]},
                "strategy": {"type": "string", "enum": ["widen_aperture_and_retry_once", "retreat_and_retry_once"]},
                "diagnosis": {"type": "string", "minLength": 8}
            },
            "required": ["object_name", "strategy", "diagnosis"],
            "additionalProperties": False
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "set_gripper_opening",
        "description": "GRIPPER-AWARE SKILL CONTRACT — Purpose: set a continuous pre-grasp finger opening in millimeters. Use only when no object is held. Expected effect: actual gripper opening converges near target_opening_mm. Expected non-effect: objects and arm should not move. After this tool, call observe_scene again and verify actual opening. This is the ONLY pre-grasp aperture-adjustment tool; do not use release-only open_gripper.",
        "parameters": {"type": "object", "properties": {"target_opening_mm": {"type": "number", "minimum": 5.0, "maximum": 100.0}}, "required": ["target_opening_mm"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "move_above",
        "description": "SKILL CONTRACT — Move TCP to a safe top-down hover about 120 mm above the named object. PRECONDITION: actual gripper opening must be within that object's recommended robust opening range from the most recent observation; otherwise prepare the gripper first. Object should remain stationary; no grasp occurs. Success: actual MuJoCo TCP reaches hover target within tolerance with no safety trip.",
        "parameters": {"type": "object", "properties": {"object_name": {"type": "string", "enum": ["red_cube", "blue_cube", "red_ball", "blue_ball"]}}, "required": ["object_name"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "approach",
        "description": "ANTI-POKE SKILL CONTRACT — Descend from HOVER_REACHED to the selected object's grasp pose. PRECONDITION: same-object hover succeeded AND current gripper opening is within the robust aperture range. This descent should be non-contact: the object should remain approximately stationary. If object motion exceeds 4 mm, treat it as premature finger/pad contact. If MoveIt blocks the Cartesian path before execution, treat that as PREGRASP_PATH_BLOCKED_REQUIRES_ADAPTATION. In either recoverable case, materially change the retry (wider aperture and/or raised grasp target), re-observe, and retry at most once before clamping.",
        "parameters": {"type": "object", "properties": {"object_name": {"type": "string", "enum": ["red_cube", "blue_cube", "red_ball", "blue_ball"]}}, "required": ["object_name"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "clamp_gripper",
        "description": "SKILL CONTRACT — Close the gripper and establish provisional two-finger contact/hold at GRASP_POSE_REACHED. Object does not need to translate yet. Success: reliable two-finger contact or hold is detected. The later lift is the physical validation that the object was actually captured.",
        "parameters": {"type": "object", "properties": {"force_n": {"type": "number", "minimum": 0.5, "maximum": 15.0}}, "required": ["force_n"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "lift",
        "description": "SKILL CONTRACT — Lift the grasped object vertically. PRECONDITION: GRASP_ESTABLISHED. TCP rises and active object MUST follow upward while maintaining approximately the same object-to-TCP relation. This validates the grasp physically.",
        "parameters": {"type": "object", "properties": {"height_m": {"type": "number", "minimum": 0.04, "maximum": 0.15}}, "required": ["height_m"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "move_above_bin",
        "description": "SKILL CONTRACT — Transport the held object to a safe release pose above the selected bin. PRECONDITION: LIFTED with an active grasped object. TCP and object move together; grasp must not be lost.",
        "parameters": {"type": "object", "properties": {"bin_name": {"type": "string", "enum": ["red_bin", "blue_bin"]}}, "required": ["bin_name"], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "open_gripper",
        "description": "RELEASE-ONLY SKILL CONTRACT — Release the active object above the target bin. PRECONDITION: BIN_HOVER_REACHED. Gripper opens, hold becomes false and object detaches/falls. Do NOT use this tool merely to prepare aperture before grasping; use set_gripper_opening for that.",
        "parameters": {"type": "object", "properties": {}, "required": [], "additionalProperties": False},
        "strict": True,
    },
    {
        "type": "function",
        "name": "verify_object_in_bin",
        "description": "SKILL CONTRACT — Deterministically verify final placement from MuJoCo coordinates after RELEASED. Read-only. Success only if the named object is actually inside the requested bin and no longer held.",
        "parameters": {"type": "object", "properties": {"object_name": {"type": "string", "enum": ["red_cube", "blue_cube", "red_ball", "blue_ball"]}, "bin_name": {"type": "string", "enum": ["red_bin", "blue_bin"]}}, "required": ["object_name", "bin_name"], "additionalProperties": False},
        "strict": True,
    },
]


class LLMAPIError(RuntimeError):
    pass


class NeroLLMAgent:
    def __init__(self, emit, tool_runner, api_base=DEFAULT_API_BASE):
        self.emit = emit
        self.tool_runner = tool_runner
        self.api_base = str(api_base).rstrip("/")
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self._last_response_id = None
        self._last_final_text = ""
        self._usage_totals = {"input_tokens": 0, "output_tokens": 0, "estimated_usd": 0.0, "requests": 0}

    @property
    def running(self):
        with self._lock:
            return bool(self._running)

    @property
    def last_response_id(self):
        return self._last_response_id

    def request_stop(self):
        self._stop_event.set()
        self.emit("[LLM STOP] Stop requested; no new LLM tool calls will be executed")

    def reset_context(self):
        if self.running:
            raise RuntimeError("Cannot reset LLM context while task is running")
        self._last_response_id = None
        self._last_final_text = ""
        self._stop_event.clear()
        self.emit("[LLM CONTEXT] Previous response/context cleared")

    @staticmethod
    def api_key_status(api_key=""):
        key = (api_key or os.environ.get("OPENAI_API_KEY", "")).strip()
        if not key:
            return False, "OPENAI_API_KEY is not configured"
        return True, "API key configured (value hidden)"

    @staticmethod
    def _resolve_key(api_key=""):
        key = (api_key or os.environ.get("OPENAI_API_KEY", "")).strip()
        if not key:
            raise LLMAPIError(
                "OPENAI_API_KEY missing. Paste it in the NERO LLM panel or export it before launch."
            )
        return key

    def _post_responses(self, payload, api_key, timeout_s=180.0):
        key = self._resolve_key(api_key)
        url = self.api_base + "/responses"
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=data,
            headers={
                "Authorization": "Bearer " + key,
                "Content-Type": "application/json",
                "User-Agent": "NERO-V2-6-Public-Release/0.1.0",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=float(timeout_s)) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")[:3000]
            raise LLMAPIError(f"OpenAI API HTTP {exc.code}: {body}") from exc
        except urllib.error.URLError as exc:
            raise LLMAPIError(f"OpenAI API network error: {exc}") from exc
        except TimeoutError as exc:
            raise LLMAPIError("OpenAI API request timed out") from exc
        try:
            obj = json.loads(raw)
        except Exception as exc:
            raise LLMAPIError("OpenAI API returned non-JSON response") from exc
        if obj.get("error"):
            raise LLMAPIError("OpenAI API error: " + json.dumps(obj["error"], ensure_ascii=False))
        return obj

    # Dollar pricing changes over time, so the public repository intentionally
    # does NOT hard-code a price table.  If desired, set NERO_LLM_PRICING_FILE
    # to a JSON file with entries of the form:
    #   {"model-id": [input_per_M, cached_input_per_M, output_per_M]}
    # Token counts remain available even when no price file is configured.
    @staticmethod
    def _load_price_map():
        path = os.environ.get("NERO_LLM_PRICING_FILE", "").strip()
        if not path:
            return {}
        try:
            with open(path, "r", encoding="utf-8") as fh:
                raw = json.load(fh)
            result = {}
            for model_id, values in (raw or {}).items():
                if isinstance(values, (list, tuple)) and len(values) == 3:
                    result[str(model_id)] = tuple(float(v) for v in values)
            return result
        except Exception:
            # Pricing is advisory only; a malformed optional file must never
            # block robot operation.  The GUI will simply report cost unknown.
            return {}

    PRICE_PER_M = _load_price_map.__func__()

    def list_available_models(self, api_key=""):
        """Return model IDs visible to the user's API key via GET /v1/models."""
        key = self._resolve_key(api_key)
        req = urllib.request.Request(
            self.api_base + "/models",
            headers={
                "Authorization": "Bearer " + key,
                "User-Agent": "NERO-V2-6-Public-Release/0.1.0",
            },
            method="GET",
        )
        try:
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                raw = resp.read().decode("utf-8", errors="replace")
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")[:3000]
            raise LLMAPIError(f"OpenAI API HTTP {exc.code}: {body}") from exc
        except urllib.error.URLError as exc:
            raise LLMAPIError(f"OpenAI API network error: {exc}") from exc
        obj = json.loads(raw)
        ids = sorted({str(x.get("id")) for x in (obj.get("data") or []) if x.get("id")})
        self.emit(f"[MODEL DISCOVERY] API key exposes {len(ids)} model IDs")
        return ids

    @staticmethod
    def _approx_tokens(text):
        # Local no-cost approximation; intentionally simple and conservative.
        return max(1, int((len(str(text)) + 3) / 4))

    @staticmethod
    def _task_object_names(task, scene_snapshot=None):
        """Resolve which canonical demo objects are still pending.

        This is intentionally deterministic rather than delegated entirely to
        the LLM.  It gives plural sorting tasks a completion guard: if the model
        tries to stop after one successful placement, the agent can inspect the
        scene and continue any requested objects that remain in ``object_zone``.

        The parser is deliberately small and demo-scene-specific.  A production
        system would replace this with a general scene graph / grounding layer.
        """
        t = str(task).lower()
        normalized = t.replace("red cube", "red_cube").replace("blue cube", "blue_cube").replace("red ball", "red_ball").replace("blue ball", "blue_ball")
        canonical = ["red_cube", "blue_cube", "red_ball", "blue_ball"]
        names = [n for n in canonical if n in normalized]
        if "all four" in t or ("all" in t and "object" in t):
            requested = list(canonical)
        elif "both ball" in t or "both balls" in t:
            requested = ["red_ball", "blue_ball"]
        elif names:
            requested = names
        elif ("objects" in t or "object" in t) and ("one by one" in t or "same color" in t or "matching" in t or "sort" in t):
            requested = list(canonical)
        else:
            return ["<one_object>"]

        # For plural tasks, estimate only objects that still need work.
        # A scene object in object_zone is pending; an object already in either
        # bin is not counted again for the matching-color sort workflow.
        if scene_snapshot and len(requested) > 1:
            objs = scene_snapshot.get("objects") or {}
            pending = [n for n in requested if (objs.get(n) or {}).get("location") == "object_zone"]
            return pending
        return requested

    def estimate_task_usage(self, task, scene_snapshot, model=DEFAULT_MODEL, reasoning_effort="low", max_tool_steps=48):
        """No-API preflight estimate for token/cost confirmation.

        This is a heuristic, not a billing quote. It estimates an iterative
        function-calling task from current prompt/tool size and number of
        manipulated objects. Dollar range uses current standard short-context
        prices when known.
        """
        object_names = self._task_object_names(task, scene_snapshot)
        n_obj = max(1, len(object_names))
        current_opening = float((scene_snapshot.get("gripper") or {}).get("actual_opening_mm") or 0.0)

        # Normal object cycle: observe + 7 manipulation/verification skills = 8.
        # V2-5 may add set_gripper_opening + re-observe. First object uses actual
        # current aperture; later cycles conservatively assume aperture prep again.
        aperture_adjustments = 0
        for i, name in enumerate(object_names):
            needs_adjust = True
            if i == 0 and name != "<one_object>":
                geom = (((scene_snapshot.get("objects") or {}).get(name) or {}).get("grasp_geometry") or {})
                rr = geom.get("recommended_opening_range_mm") or []
                if len(rr) == 2:
                    needs_adjust = not (float(rr[0]) <= current_opening <= float(rr[1]))
            if needs_adjust:
                aperture_adjustments += 1

        expected_tool_steps = min(int(max_tool_steps), 8 * n_obj + 2 * aperture_adjustments)
        expected_requests = max(2, expected_tool_steps + 1)

        base_context = SYSTEM_INSTRUCTIONS + json.dumps(TOOLS, ensure_ascii=False) + str(task)
        base_tokens = self._approx_tokens(base_context)
        # Tool outputs / state observations make each continuation larger.
        incremental_tokens = 520
        input_total = sum(base_tokens + incremental_tokens * i for i in range(expected_requests))
        per_req_out = {
            "none": 20, "low": 40, "medium": 90,
            "high": 180, "xhigh": 320, "max": 500,
        }.get(str(reasoning_effort).lower(), 90)
        output_total = expected_requests * per_req_out

        prices = self.PRICE_PER_M.get(str(model))
        likely_cost = None
        conservative_cost = None
        if prices:
            pin, pcached, pout = prices
            # Conservative upper estimate: treat all input as uncached.
            conservative_cost = (input_total * pin + output_total * pout) / 1_000_000.0
            # Likely estimate: first request uncached; ~75% of later repeated input cached.
            first = base_tokens
            later = max(0, input_total - first)
            cached = int(later * 0.75)
            uncached = input_total - cached
            likely_cost = (uncached * pin + cached * pcached + output_total * pout) / 1_000_000.0

        return {
            "model": str(model),
            "reasoning_effort": str(reasoning_effort),
            "objects_estimated": n_obj,
            "aperture_adjustments_estimated": aperture_adjustments,
            "tool_steps_estimated": expected_tool_steps,
            "api_requests_estimated": expected_requests,
            "input_tokens_estimated": int(input_total),
            "output_tokens_estimated": int(output_total),
            "likely_cost_usd": None if likely_cost is None else round(likely_cost, 5),
            "conservative_cost_usd": None if conservative_cost is None else round(conservative_cost, 5),
            "pricing_known": bool(prices),
            "note": "Heuristic preflight estimate only; actual usage depends on model reasoning, retries, caching, and tool outputs.",
        }

    def _log_usage(self, response, model):
        usage = response.get("usage") or {}
        in_tok = int(usage.get("input_tokens") or 0)
        out_tok = int(usage.get("output_tokens") or 0)
        cached = int((usage.get("input_tokens_details") or {}).get("cached_tokens") or 0)
        prices = self.PRICE_PER_M.get(str(model))
        if prices:
            pin, pcached, pout = prices
            uncached = max(0, in_tok - cached)
            estimated = (uncached * pin + cached * pcached + out_tok * pout) / 1_000_000.0
            self._usage_totals["estimated_usd"] += estimated
            price_text = f"estimated_request=${estimated:.5f} | estimated_session=${self._usage_totals['estimated_usd']:.5f}"
        else:
            price_text = "cost=unavailable (unknown model rate)"
        self._usage_totals["requests"] += 1
        self._usage_totals["input_tokens"] += in_tok
        self._usage_totals["output_tokens"] += out_tok
        self.emit(f"[MODEL USAGE] model={model} | input={in_tok} | cached_input={cached} | output={out_tok} | {price_text} | session_requests={self._usage_totals['requests']}")

    @staticmethod
    def _extract_text(response):
        parts = []
        for item in response.get("output", []) or []:
            if item.get("type") != "message":
                continue
            for content in item.get("content", []) or []:
                if content.get("type") == "output_text" and content.get("text"):
                    parts.append(str(content.get("text")))
                elif content.get("type") == "refusal" and content.get("refusal"):
                    parts.append(str(content.get("refusal")))
        return "\n".join(parts).strip()

    @staticmethod
    def _function_calls(response):
        return [x for x in (response.get("output", []) or []) if x.get("type") == "function_call"]

    def test_connection(self, model=DEFAULT_MODEL, api_key="", reasoning_effort="low"):
        self.emit(f"[LLM TEST] Sending API connectivity test | model={model}")
        t0 = time.perf_counter()
        payload = {
            "model": str(model),
            "instructions": "Reply with exactly NERO_LLM_READY.",
            "input": "Connectivity test only. Do not call tools.",
            "reasoning": {"effort": str(reasoning_effort)},
            "max_output_tokens": 32,
            "store": True,
        }
        response = self._post_responses(payload, api_key)
        self._log_usage(response, model)
        text = self._extract_text(response)
        self._last_response_id = response.get("id")
        self.emit(
            f"[LLM TEST OK] id={self._last_response_id} | "
            f"elapsed={time.perf_counter()-t0:.2f}s | reply={text!r}"
        )
        return text

    def plan_only(self, task, scene_snapshot, model=DEFAULT_MODEL, api_key="", reasoning_effort="medium"):
        task = str(task).strip()
        if not task:
            raise ValueError("Task is empty")
        self.emit(f"[LLM PLAN ONLY] task={task!r} | model={model} | effort={reasoning_effort}")
        payload = {
            "model": str(model),
            "instructions": (
                SYSTEM_INSTRUCTIONS
                + "\nPLAN-ONLY MODE: Do not call any tools. Return a concise numbered sequence of the robot tools you would call, including verification steps."
            ),
            "input": "TASK:\n" + task + "\n\nCURRENT SCENE SNAPSHOT:\n" + json.dumps(scene_snapshot, ensure_ascii=False, indent=2),
            "reasoning": {"effort": str(reasoning_effort)},
            "max_output_tokens": 1000,
            "store": True,
        }
        t0 = time.perf_counter()
        response = self._post_responses(payload, api_key)
        self._log_usage(response, model)
        text = self._extract_text(response)
        self._last_response_id = response.get("id")
        self._last_final_text = text
        self.emit(f"[LLM PLAN] elapsed={time.perf_counter()-t0:.2f}s\n{text}")
        return text

    def run_task(
        self,
        task,
        model=DEFAULT_MODEL,
        api_key="",
        reasoning_effort="medium",
        max_tool_steps=48,
        dry_run=False,
    ):
        """Execute one natural-language task as a bounded tool-calling episode.

        ``run_task`` is an orchestrator, not a motion controller.  Each tool call
        is serialized, executed by the deterministic backend, and returned to the
        model together with measured postconditions.  The loop is bounded by
        ``max_tool_steps`` and by one-retry recovery budgets enforced downstream.
        """
        task = str(task).strip()
        if not task:
            raise ValueError("Task is empty")
        max_tool_steps = max(1, min(60, int(max_tool_steps)))

        with self._lock:
            if self._running:
                raise RuntimeError("LLM agent is already running")
            self._running = True

        self._stop_event.clear()
        self._last_response_id = None
        self._last_final_text = ""
        t_task = time.perf_counter()
        tool_count = 0
        multi_object_guard_nudges = 0

        try:
            ok, key_status = self.api_key_status(api_key)
            if not ok:
                raise LLMAPIError(key_status)
            self.emit(
                f"[LLM TASK START] model={model} | effort={reasoning_effort} | "
                f"max_tool_steps={max_tool_steps} | dry_run={bool(dry_run)}"
            )
            self.emit(f"[LLM USER TASK] {task}")

            payload = {
                "model": str(model),
                "instructions": SYSTEM_INSTRUCTIONS,
                "input": task,
                "tools": TOOLS,
                "tool_choice": "auto",
                "parallel_tool_calls": False,
                "reasoning": {"effort": str(reasoning_effort)},
                "max_output_tokens": 1200,
                "store": True,
            }

            request_index = 0
            while True:
                if self._stop_event.is_set():
                    raise RuntimeError("LLM task stopped by user")

                request_index += 1
                self.emit(
                    f"[LLM REQUEST #{request_index}] "
                    + ("initial task" if not self._last_response_id else f"continue from {self._last_response_id}")
                )
                t_req = time.perf_counter()
                response = self._post_responses(payload, api_key)
                self._log_usage(response, model)
                self._last_response_id = response.get("id")
                calls = self._function_calls(response)
                text = self._extract_text(response)
                self.emit(
                    f"[LLM RESPONSE #{request_index}] id={self._last_response_id} | "
                    f"elapsed={time.perf_counter()-t_req:.2f}s | tool_calls={len(calls)}"
                )
                if text:
                    self.emit(f"[LLM TEXT] {text}")

                if not calls:
                    # V2-6 deterministic completion guard for plural tasks.
                    # A small/fast model may otherwise summarize after finishing only
                    # one object. Read the scene directly and refuse premature task
                    # completion while requested objects remain in object_zone.
                    requested_all = self._task_object_names(task, None)
                    is_multi_object = bool(len(requested_all) > 1 and "<one_object>" not in requested_all)
                    if is_multi_object and multi_object_guard_nudges < 4:
                        try:
                            guard_result = self.tool_runner("observe_scene", {})
                            guard_scene = guard_result.get("scene") if isinstance(guard_result, dict) else None
                            pending = self._task_object_names(task, guard_scene or {})
                        except Exception as guard_exc:
                            pending = []
                            self.emit(f"[MULTI-OBJECT GUARD WARN] scene check failed: {guard_exc}")
                        if pending:
                            multi_object_guard_nudges += 1
                            self.emit(
                                f"[MULTI-OBJECT GUARD] model attempted to finish early; "
                                f"pending={pending} | nudge={multi_object_guard_nudges}/4"
                            )
                            payload = {
                                "model": str(model),
                                "instructions": SYSTEM_INSTRUCTIONS,
                                "previous_response_id": self._last_response_id,
                                "input": (
                                    "NERO V2-6 deterministic task-completion guard: the user's plural sorting "
                                    f"task is NOT complete. Objects still in object_zone: {pending}. "
                                    "Continue with the next pending object using the normal full pick/place cycle. "
                                    "Do not give a final summary while any requested object remains pending. "
                                    "If one object has exhausted its allowed recovery budget, skip that object and "
                                    "continue other pending objects, then report the failed object at the end."
                                ),
                                "tools": TOOLS,
                                "tool_choice": "auto",
                                "parallel_tool_calls": False,
                                "reasoning": {"effort": str(reasoning_effort)},
                                "max_output_tokens": 1200,
                                "store": True,
                            }
                            continue

                    self._last_final_text = text
                    self.emit(
                        f"[LLM TASK COMPLETE] tool_steps={tool_count} | "
                        f"elapsed={time.perf_counter()-t_task:.2f}s"
                    )
                    if text:
                        self.emit(f"[LLM FINAL] {text}")
                    return text

                outputs = []
                for call in calls:
                    if self._stop_event.is_set():
                        raise RuntimeError("LLM task stopped by user")
                    tool_count += 1
                    if tool_count > max_tool_steps:
                        raise RuntimeError(
                            f"LLM exceeded max_tool_steps={max_tool_steps}; stopped before executing another tool"
                        )

                    name = str(call.get("name", ""))
                    call_id = str(call.get("call_id", ""))
                    raw_args = call.get("arguments", "{}") or "{}"
                    try:
                        args = json.loads(raw_args)
                        if not isinstance(args, dict):
                            raise ValueError("arguments must be a JSON object")
                    except Exception as exc:
                        result = {"ok": False, "error": f"Invalid tool arguments: {exc}", "raw": raw_args}
                    else:
                        self.emit(
                            f"[LLM CALL #{tool_count}] {name}("
                            + ", ".join(f"{k}={v!r}" for k, v in args.items())
                            + ")"
                        )
                        if dry_run and name not in ("observe_scene", "verify_object_in_bin"):
                            result = {"ok": True, "dry_run": True, "tool": name, "arguments": args}
                            self.emit(f"[LLM DRY RUN] skipped physical tool {name}")
                        else:
                            t_tool = time.perf_counter()
                            try:
                                result = self.tool_runner(name, args)
                                if not isinstance(result, dict):
                                    result = {"ok": bool(result), "result": result}
                            except Exception as exc:
                                result = {"ok": False, "tool": name, "error": str(exc)}
                            self.emit(
                                f"[LLM TOOL RESULT #{tool_count}] {name} | "
                                f"elapsed={time.perf_counter()-t_tool:.2f}s | "
                                + json.dumps(result, ensure_ascii=False, separators=(",", ":"))
                            )

                    outputs.append(
                        {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps(result, ensure_ascii=False),
                        }
                    )

                payload = {
                    "model": str(model),
                    "instructions": SYSTEM_INSTRUCTIONS,
                    "previous_response_id": self._last_response_id,
                    "input": outputs,
                    "tools": TOOLS,
                    "tool_choice": "auto",
                    "parallel_tool_calls": False,
                    "reasoning": {"effort": str(reasoning_effort)},
                    "max_output_tokens": 1200,
                    "store": True,
                }

        except Exception as exc:
            self.emit(f"[LLM TASK FAILED] {exc}")
            raise
        finally:
            with self._lock:
                self._running = False
            self.emit(f"[LLM TASK END] total_elapsed={time.perf_counter()-t_task:.2f}s")

# Backward compatibility for private/local V2-5.x launchers that imported the
# historical class name.  New public code should import ``NeroLLMAgent``.
NeroAstraAgent = NeroLLMAgent
