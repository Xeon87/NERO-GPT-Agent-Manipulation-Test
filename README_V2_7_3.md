# NERO V2-7.3 — Late-Settle Verification + Task-End Summary

This is an incremental patch on the validated local V2-7.2 runtime.

## Why V2-7.3 exists

In the V2-7.2 egg test, `move_to_pregrasp` was marked failed because the initial physical joint/TCP verification timed out, even though MuJoCo reached the planned pregrasp pose roughly one second later. The backend then incorrectly treated this as a grasp-candidate failure and selected a second grasp candidate.

V2-7.3 separates **execution-layer tracking/settling failures** from **grasp-geometry failures**.

## Changes

- Frozen V69 remains byte-for-byte unchanged.
- Wrapper-only passive late-settle recheck extended from 1.8 s to 4.5 s.
- If TCP enters tolerance during late-settle recheck, the motion becomes success-with-warning and the same grasp plan continues to `approach_planned_grasp`.
- If `PREGRASP_PHYSICAL_TRACKING_TIMEOUT` remains after the extended wait, the candidate is **not** marked exhausted and the LLM is told to stop rather than select another candidate.
- Existing V2-7.2 egg identity labels 1/2/3/4, hold-state continuity, grasp-loss recovery, bounded wrist reorientation and Copy Full Log are retained.
- After every `[LLM TASK END]`, a deterministic high-level task summary is printed as the final log record:
  - **Green**: physically verified success with no warnings/failures.
  - **Yellow**: success with warnings/recovered failures, or episode ended without full physical verification.
  - **Red**: failed task or exception.
- Summary includes task, final outcome, tool count, failed-tool count, warning count, physical verification count and last failure reason.

## Run

```bash
chmod +x start_nero_v2_7_3.sh
./start_nero_v2_7_3.sh
```

The installer reads:

```text
~/mujoco_ws/nero/NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation
```

and creates a separate runtime:

```text
~/mujoco_ws/nero/NERO_V2_7_3_Late_Settle_Task_Summary
```

The V2-7.2 directory is not overwritten.
