# NERO V2-13 — orient_for_slot J7 Redundancy Redistribution

V2-13 changes only the `orient_for_slot()` transport logic on top of the V2-12 frontier-diagnostic baseline.

- Preserve V2-12's 2.5 mm vertical-then-horizontal incremental route.
- Monitor J7 hard-limit margin.
- Below 18 deg margin, search collision-checked IK solutions for the same TCP XYZ and same TCP quaternion using inward-shifted J7 seeds.
- Accept only small local branch changes and target roughly 28 deg J7 margin.
- If a J7 joint-limit frontier is hit, force one redistribution attempt and retry the blocked probe.
- If recovery fails, retain V2-12 behavior: execute only the validated prefix and stop before the rejected waypoint.

Frozen/unchanged: Agent, V69, V2-7.2 launcher, grasp, clamp, lift, Step-B bounded wrist reorientation, slot entry, insertion/release, GUI and MuJoCo startup.

Static/build validation passed against the saved V2-12 backend snapshot. Local Ubuntu/MuJoCo runtime behavior requires an actual run.