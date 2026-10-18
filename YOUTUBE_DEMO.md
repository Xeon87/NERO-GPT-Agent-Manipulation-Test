# YouTube Demo Checklist

## Recommended video structure (60–120 s)

**0–8 s — Title / architecture**  
`NERO V2-6 — LLM-Guided Manipulation with Skill Contracts & Failure-Aware Recovery`

**8–15 s — Show the scene**  
Briefly show MuJoCo, the NERO arm, colored objects/bins, and the GUI.

**15–25 s — Show the command**

```text
Put the red objects to the bin with the same color, one by one.
```

Show the selected LLM model in the GUI, but describe it as a **configurable planner**, not as the core robot controller.

**25–100 s — Let the task run**  
Keep MuJoCo and the most important log region visible. Useful log lines are:

- `observe_scene`
- gripper prior / target aperture
- `move_above`
- `approach`
- `PHYSICAL VERIFY`
- `lift`
- `move_above_bin`
- final placement verification
- next-object continuation

**Final 10 s — Limits / next step**

Overlay something factual such as:

```text
Current: MuJoCo + top-down 2-finger grasping + hand-crafted manipulation priors
Next: real NERO + D435i + tactile feedback / learned manipulation priors
```

## Suggested title

`NERO V2-6: LLM-Guided Robot Manipulation with Physical Verification and Failure Recovery`

## Suggested short description

NERO connects a configurable LLM task planner to deterministic robot skills in MuJoCo/ROS2. The LLM selects high-level skills, while MoveIt/BioIK/TRAC-IK, skill contracts, physical postcondition checks, manipulation priors, and bounded failure recovery remain in the robot layer.

## Avoid overstating

Do not describe this release as a general-purpose VLA, learned manipulation policy, or autonomous real-world robot. The strongest story is the **clean systems architecture and inspectable closed-loop execution**.
