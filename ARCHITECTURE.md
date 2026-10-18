# Architecture

NERO deliberately separates probabilistic task reasoning from deterministic robot execution.

```mermaid
flowchart TB
    A[User natural-language instruction]
    B[Configurable LLM Task Planner]
    C[Skill Contract Layer]
    D[Manipulation Prior & Grasp Planning]
    E[Motion Execution: MoveIt + BioIK / TRAC-IK]
    F[Robot & Simulation: MuJoCo + ROS2]
    G[Physical Verification & State Update]
    H[Failure-Aware Recovery]
    I[Scene State / Object Geometry]

    A --> B
    I --> B
    B --> C
    C --> D
    I --> D
    D --> E
    E --> F
    F --> G
    G --> B
    E -. pre-execution planning block .-> H
    F -. safety / contact evidence .-> H
    H --> I
```

## Control boundary

The LLM may choose **what skill to call and with what high-level argument**. It may not send joint angles or bypass safety state. The backend validates state transitions and measures whether the physical postcondition was actually achieved.

## Skill state machine

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> OBSERVED: observe_scene
    OBSERVED --> HOVER_REACHED: move_above
    HOVER_REACHED --> GRASP_POSE_REACHED: approach
    GRASP_POSE_REACHED --> GRASP_ESTABLISHED: clamp_gripper
    GRASP_ESTABLISHED --> LIFTED: lift
    LIFTED --> BIN_HOVER_REACHED: move_above_bin
    BIN_HOVER_REACHED --> RELEASED: open_gripper
    RELEASED --> PLACEMENT_VERIFIED: verify_object_in_bin
    PLACEMENT_VERIFIED --> OBSERVED: next object
    HOVER_REACHED --> SAFETY_STOPPED: physical pre-grasp safety event
    SAFETY_STOPPED --> OBSERVED: explicit recovery + fresh perception
```

A planning refusal before execution is handled separately because no physical contact has occurred; it does not require clearing a safety latch.

## Manipulation prior

The current prior is deliberately simple and interpretable. It expands the pre-grasp aperture beyond nominal object width and uses different clearances for boxes and spheres. Retry bonuses are task-local and are cleared at a fresh episode boundary.

This is a useful placeholder for future tactile- or data-driven priors because every chosen aperture, geometry estimate, failure, and outcome is already observable in the log.
