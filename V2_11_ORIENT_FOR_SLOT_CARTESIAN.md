# NERO V2-11 — orient_for_slot Cartesian Transport Only

Base: V2-10.

Only behavior change vs V2-10:

- Replace V2-10 repeated `move_to_pose()` transport inside `orient_for_slot()`.
- Build one continuous MoveIt `GetCartesianPath` trajectory.
- First rise vertically at current XY, then translate in XY at safe Z.
- Explicit waypoints are spaced at <=25 mm.
- Cartesian interpolation max step = 3 mm.
- Max Cartesian speed = 35 mm/s.
- The actual TCP quaternion measured at the start of `orient_for_slot()` stays fixed during transport.
- Plan once and execute once to avoid repeated controller synchronization/global pose IK branch reselection.

Unchanged:

- Agent / LLM orchestration
- MuJoCo / Viewer startup
- frozen V69 source
- V2-7.2 runtime launcher
- grasp planning / approach / clamp / lift
- inherited V2-7.2 bounded wrist reorientation Step B
- move_to_slot_entry
- guarded_insert_and_release
- verification and V2-9 log/summary styling
