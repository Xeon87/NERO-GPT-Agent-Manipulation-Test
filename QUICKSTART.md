# Quick Start

## 1. Prerequisites

The launcher expects the same local stack used by the validated NERO simulation:

```text
ROS_DISTRO      humble
AGX_ARM_WS      ~/agx_arm_ws
NERO_WS         ~/mujoco_ws/nero
VENV            ~/mujoco_ws/venv
BIO_IK_WS       ~/bio_ik_ws
TRAC_IK_WS      ~/trac_ik_ws
```

Each path can be overridden with an environment variable.

## 2. API key and model

```bash
export OPENAI_API_KEY="..."
export NERO_LLM_MODEL="gpt-5.6-luna"   # example only; GUI remains editable
```

Do not commit `.env` files or API keys.

## 3. Launch

Close any older NERO launcher first so two MuJoCo/ROS backends do not compete for the same topics/controllers.

```bash
cd NERO-GPT-Agent-Manipulation-Test
bash start_nero_v2_6.sh
```

The launcher copies the V2-6 Python files into `$NERO_WS` and reuses the existing `nero_motor.xml` and AgileX/ROS assets.

## 4. Recommended first demo

1. Press **ZERO ALL**.
2. Confirm the log reports a fresh episode reset.
3. Run:

```text
Put the red objects to the bin with the same color, one by one.
```

4. Watch the log for the complete skill sequence and final placement verification.

## 5. Between experiments

You do not need to restart the GUI after every run. With no held object:

```text
ZERO ALL -> new RUN TASK
```

starts a fresh logical episode and clears task-local retry/adaptation state.
