# NERO V2-7 — Constraint-Aware Grasp Planning

This experimental package is a small-step upgrade from the validated **NERO V2-6 Egg Task Semantic Integration** line.

## Scope

- Frozen V69 motion core remains unchanged.
- Keeps the marble tabletop, wood floor, egg basket, four rigid eggs, eight-slot egg holder, Egg Semantic Layer, and existing egg placement flow from V2-6 Egg Semantic Integration.
- Adds constraint-aware egg grasp planning before clamp/lift.
- Uses egg orientation, multiple bounded grasp candidates, basket-rim clearance checks, candidate exhaustion, and explicit failure when object reorientation is required.
- Does **not** add egg breaking, tactile damage, push/roll object reorientation, or a major architecture rewrite.

## One-shot start

The launcher automatically finds the already-tested V2-6 Egg Semantic package at:

`~/mujoco_ws/nero/NERO_V2_6_Egg_Task_Semantic_Integration`

Then it generates and launches:

`~/mujoco_ws/nero/NERO_V2_7_Constraint_Aware_Grasp_Planning`

Run after extracting this ZIP:

```bash
chmod +x start_nero_v2_7.sh
./start_nero_v2_7.sh
```

If your V2-6 Egg Semantic folder is elsewhere:

```bash
NERO_V26_EGG_DIR=/your/path/NERO_V2_6_Egg_Task_Semantic_Integration ./start_nero_v2_7.sh
```

## First test

`Pick up egg_1 and place it into the first empty egg-holder slot.`

## Frozen V69

Expected SHA256:

`3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7`
