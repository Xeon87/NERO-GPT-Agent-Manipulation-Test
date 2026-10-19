# NERO V2-7 — Constraint-Aware Grasp Planning

This experimental branch is intentionally **not** a redesign of NERO. It upgrades the V2-6 Egg Task Semantic Integration while keeping the frozen V69 motion core unchanged.

## Scope

V2-7 adds a grasp-planning layer before the existing clamp/lift flow:

`observe_scene -> set_gripper_opening -> plan_grasp -> move_to_pregrasp -> approach_planned_grasp -> clamp_gripper -> lift -> existing egg-holder placement flow`

The planner:

- reads the egg quaternion and estimates the world-space long axis;
- prefers a wrist yaw that aligns the finger-pad long axis with the egg major axis, so closing occurs across the egg short axis;
- handles near-upright eggs with a basket-tangent fallback because horizontal egg yaw is underdetermined;
- models the current two-finger projected footprint against the circular wicker basket rim;
- generates bounded yaw alternatives (+/-15 deg, +/-30 deg, orthogonal fallback) and small inward grasp-center offsets;
- rejects geometrically unsafe candidates before motion;
- MoveIt-preflights remaining pregrasp poses without execution;
- marks a candidate exhausted after a blocked/contacting attempt and never repeats it;
- returns `REORIENTATION_REQUIRED_BUT_NOT_IMPLEMENTED_V2_7` when every bounded candidate is exhausted/infeasible instead of entering an infinite retry loop.

## Explicitly deferred

V2-7 does **not** yet add:

- egg breaking/damage;
- tactile feedback;
- push/roll/nudge object reorientation;
- learned grasp proposals;
- a large redesign of the Skill/Skill Contract architecture.

Those are intentionally left for later versions so this step isolates constraint-aware grasp planning.

## Frozen V69

Expected SHA256:

`3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7`

The upgrader refuses to proceed if the local V69 file differs.

## Build V2-7 from the latest V2-6 Egg Semantic package

The branch contains a local upgrader under `tools/`. It takes the already generated `NERO_V2_6_Egg_Task_Semantic_Integration` folder and creates a sibling `NERO_V2_7_Constraint_Aware_Grasp_Planning` folder, preserving all other package files.

First test command:

`Pick up egg_1 and place it into the first empty egg-holder slot.`

For the first run, inspect the log entries beginning with `[V2-7 GRASP PLAN]`, `[V2-7 CANDIDATE BLOCKED]`, or `[V2-7 GRASP INFEASIBLE]` before tuning geometry parameters.
