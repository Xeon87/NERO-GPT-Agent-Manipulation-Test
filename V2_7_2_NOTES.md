# NERO V2-7.2 — Hold Continuity + Bounded Wrist Reorientation

This is a small-step upgrade from the validated local V2-7.1 runtime.

## Scope

- **Frozen V69 remains byte-for-byte unchanged.**
- Preserve V2-7 constraint-aware egg grasp planning.
- Preserve V2-7.1 placement planning and `Copy Full Log` UI.
- Do not add egg breaking, tactile damage, push/roll/nudge, or a new general skill architecture.

## V2-7.2 changes

1. **Physical egg identity labels**
   - `obj_egg1` / `obj_egg2` / `obj_egg3` / `obj_egg4` receive visible dark 3D numerals `1/2/3/4`.
   - Labels are child geoms of the actual egg bodies and are duplicated on both local-X sides.
   - Startup checks require exactly four named egg bodies before simulation continues.

2. **HOLD-state continuity**
   - Before a MuJoCo HOLD equality is disabled, the current displayed world pose of the held free body is captured.
   - After `eq_active=0`, the free-joint qpos is explicitly re-seeded to that current pose.
   - Release velocity is zeroed and gravity then acts normally.
   - This is intended to prevent a stale pre-grasp free-joint pose from reappearing as an apparent teleport back to the basket.

3. **Early stale-state recovery**
   - If `hold_active=false` while the logical state still says `LIFTED`, `PLACEMENT_ORIENTED`, `SLOT_ENTRY_ALIGNED`, etc., the stale active-object/placement state is cleared **before** an older `observe_scene` handler can return.

4. **Bounded held-egg reorientation**
   - A ~90° egg reorientation is split into ~10° steps instead of 1–3 large pose moves.
   - Each step is MoveIt-planned **before execution**.
   - The final joint solution is inspected against per-step limits.
   - Large J1–J3 and excessive J4/J5 branch changes are rejected before they reach the controller.
   - J6/J7 are intentionally much cheaper in the selection score, while not being hard-coded to fixed angles.
   - Orientation speed/acceleration are reduced.

## First test

Use one egg first, preferably the previously successful `egg_3`:

`Pick up egg_3 and place it into the first empty egg-holder slot.`

Watch for:

- the visible numeral on the held egg;
- `[V2-7.2 WRIST PREFLIGHT OK]` before each small orientation move;
- no safety trip during reorientation;
- if the egg is lost, whether the **same numbered egg** falls near the current safe workspace rather than reappearing in the basket;
- `[V2-7.2 GRASP LOST RECOVERY]` followed by a clean observed state rather than stale `LIFTED`.

The build environment here cannot execute the user's ROS2 + MuJoCo hardware stack, so runtime physical behavior still requires validation on the Ubuntu NERO machine.
