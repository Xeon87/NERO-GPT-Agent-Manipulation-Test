# NERO V2-7.1 — Placement-Aware Egg Handling

V2-7.1 is a deliberately small-step upgrade from the validated V2-7 constraint-aware egg grasping runtime.

## Frozen boundary

- `NERO_V69_ReachableZone_BioIK_Planner_Core.py` remains byte-for-byte frozen.
- Egg breaking, tactile damage, push/roll/nudge reorientation, and generic LLM-generated skill synthesis remain out of scope.
- V2-7 grasp planning remains the pickup path.

## New egg placement path

```text
observe_scene
→ set_gripper_opening
→ plan_grasp
→ move_to_pregrasp
→ approach_planned_grasp
→ clamp_gripper
→ lift
→ plan_placement
→ orient_for_slot
→ move_to_slot_entry
→ guarded_insert_and_release
→ verify_object_in_slot
```

For an active egg, legacy `move_above_receptacle`, `place_into_receptacle`, and direct `open_gripper` placement are deterministically blocked.

## Long-axis placement

The egg is still grasped across its short axis. After lift, `plan_placement` reads the actual held-egg quaternion and computes the egg long axis relative to the egg-holder slot entry axis.

`orient_for_slot` first transports the held egg to a high, open workspace waypoint. Only there does it reorient the held object. The TCP position is kept approximately fixed during 1–3 bounded orientation steps. BioIK is seeded from the current physical pose, so wrist-dominant motion is preferred by minimum displacement, but Joint 6 / Joint 7 are not hard-coded.

Acceptance target for the first experiment:

- egg long-axis error to slot axis: <= 10 deg
- hold remains active
- no safety-trip increment

## Precise slot entry

`move_to_slot_entry` preserves the aligned orientation and centers the held egg over the chosen slot.

- target egg-center XY error: <= 7 mm
- one deterministic micro-correction maximum
- no LLM oscillation around the slot mouth

## Slow axial insertion and gentle release

`guarded_insert_and_release` descends along the aligned egg/slot axis in bounded segments:

- 10 mm coarse steps
- 5 mm fine steps near release
- abort if hold is lost
- abort if safety trip changes
- abort if egg-center XY error exceeds 12 mm
- abort if long-axis error exceeds 12 deg

The first V2-7.1 test uses a shallow 8 mm slot capture depth. The egg is released after its lower end is geometrically captured by the cup, before the gripper fingers need to descend deeply into the holder.

Release order is intentionally conservative:

1. leave clamp/force mode;
2. open only about 10 mm at low height, enough to detach the egg;
3. let the egg settle briefly;
4. retreat the partially-open gripper vertically about 80 mm;
5. only after the gripper clears the holder, command the wide opening;
6. call `verify_object_in_slot` for the final placement verdict.

## Bystander Egg Guard

At `plan_grasp`, V2-7.1 records the other eggs. Before clamp, any non-target egg that moved more than 3 mm causes:

`BYSTANDER_EGG_DISTURBED`

The selected grasp candidate is marked tried and the robot does not clamp/lift that candidate.

## Hold-loss state recovery

If a placement-stage observation finds `hold_active=false`, V2-7.1 clears the stale placement state instead of remaining stuck in `RECEPTACLE_HOVER_REACHED` / placement state.

## GUI

A `Copy Full Log` button is added immediately above the persistent Unified Runtime Log. It copies the entire log to the system clipboard.

## First test

Use one egg first:

```text
Pick up egg_3 and place it into the first empty egg-holder slot.
```

The most useful log sections are:

- `[V2-7 GRASP PLAN]`
- `[V2-7.1 PLACEMENT PLAN]`
- `[V2-7.1 ORIENT OK]`
- `[V2-7.1 SLOT ENTRY OK]`
- `[V2-7.1 INSERT STEP]`
- `[V2-7.1 RELEASE COMPLETE]`
- final `verify_object_in_slot`
