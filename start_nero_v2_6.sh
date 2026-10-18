#!/usr/bin/env bash
set -eo pipefail

# ============================================================
# NERO V2-6 Public Release launcher
#
# This script intentionally reuses an existing, working NERO/AgileX workspace
# instead of redistributing vendor meshes, ROS packages, or local robot assets.
# It installs the public-release Python files into $NERO_WS, verifies the
# required solver stack, starts MuJoCo + the unified backend, launches the
# primary BioIK MoveIt instance and the namespaced TRAC-IK reference instance,
# and restores kinematics.yaml on shutdown.
#
# Override paths through environment variables (NERO_WS, AGX_ARM_WS, etc.) so
# the repository contains no user-specific absolute paths.
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
AGX_ARM_WS="${AGX_ARM_WS:-$HOME/agx_arm_ws}"
NERO_WS="${NERO_WS:-$HOME/mujoco_ws/nero}"
VENV="${VENV:-$HOME/mujoco_ws/venv}"
BIO_IK_WS="${BIO_IK_WS:-$HOME/bio_ik_ws}"
TRAC_IK_WS="${TRAC_IK_WS:-$HOME/trac_ik_ws}"
USE_RVIZ="${USE_RVIZ:-true}"

BIO_IK_MODE="${BIO_IK_MODE:-gd_c}"
BIO_IK_TIMEOUT="${BIO_IK_TIMEOUT:-0.05}"
BIO_MIN_DISP_W="${BIO_MIN_DISP_W:-1.0}"
BIO_AVOID_LIMIT_W="${BIO_AVOID_LIMIT_W:-0.4}"
BIO_CENTER_W="${BIO_CENTER_W:-0.1}"
TRAC_IK_SOLVE_TYPE="${TRAC_IK_SOLVE_TYPE:-Distance}"
TRAC_IK_TIMEOUT="${TRAC_IK_TIMEOUT:-0.10}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
AGX_SETUP="${AGX_ARM_WS}/install/setup.bash"
BIO_SETUP="${BIO_IK_WS}/install/setup.bash"
TRAC_SETUP="${TRAC_IK_WS}/install/setup.bash"
VENV_ACTIVATE="${VENV}/bin/activate"
BACKEND="${NERO_WS}/NERO_V2_6_Unified_Backend.py"
AGENT="${NERO_WS}/NERO_V2_6_Agent.py"
CORE="${NERO_WS}/NERO_V69_ReachableZone_BioIK_Planner_Core.py"

# If launched from an extracted download folder, install the V2-6 public-release files
# into the user's established NERO workspace first. Existing nero_motor.xml,
# AgileX meshes/workspaces and ROS setup are intentionally reused.
mkdir -p "${NERO_WS}"
for packaged in NERO_V2_6_Unified_Backend.py NERO_V2_6_Agent.py NERO_V69_ReachableZone_BioIK_Planner_Core.py; do
    src="${SCRIPT_DIR}/${packaged}"
    dst="${NERO_WS}/${packaged}"
    if [[ -f "${src}" ]]; then
        if [[ "$(readlink -f "${src}")" != "$(readlink -f "${dst}")" ]]; then
            cp -f "${src}" "${dst}"
        fi
    fi
done

MODEL_FILE="${NERO_WS}/nero_motor.xml"
[[ -f "${MODEL_FILE}" ]] || {
    echo "[FAIL] Existing NERO model not found: ${MODEL_FILE}" >&2
    echo "       Keep your established ~/mujoco_ws/nero workspace; V2-6 reuses its nero_motor.xml and AgileX assets." >&2
    exit 1
}

LOG_DIR="${NERO_WS}/logs"
mkdir -p "${LOG_DIR}"
BACKEND_LOG="${LOG_DIR}/v2_6_backend.log"
BIO_MOVEIT_LOG="${LOG_DIR}/v2_6_bio_moveit.log"
TRAC_MOVEIT_LOG="${LOG_DIR}/v2_6_trac_ref_moveit.log"
STATUS_LOG="${LOG_DIR}/v2_6_startup_status.log"
KIN_BACKUP="${LOG_DIR}/v2_6_kinematics_backup.yaml"
: > "${STATUS_LOG}"
export NERO_STATUS_FILE="${STATUS_LOG}"

BACKEND_PID=""
BIO_MOVEIT_PID=""
TRAC_MOVEIT_PID=""
KIN_YAML=""
KIN_BACKED_UP=0

stamp(){ date '+%H:%M:%S'; }
status(){ local level="$1"; shift; local line="[$(stamp)] [${level}] $*"; echo "$line"; echo "$line" >> "$STATUS_LOG"; }
fatal(){ status FAIL "$1"; exit 1; }

restore_kinematics(){
    set +e
    if [[ "$KIN_BACKED_UP" -eq 1 && -n "$KIN_YAML" && -f "$KIN_BACKUP" ]]; then
        cp -f "$KIN_BACKUP" "$KIN_YAML" 2>/dev/null || true
        status INFO "Restored original AgileX kinematics.yaml"
        KIN_BACKED_UP=0
    fi
}

cleanup(){
    set +e
    status INFO "Shutdown requested; stopping NERO V2-6 processes"
    if [[ -n "$TRAC_MOVEIT_PID" ]]; then
        kill -- "-$TRAC_MOVEIT_PID" 2>/dev/null || kill "$TRAC_MOVEIT_PID" 2>/dev/null || true
    fi
    if [[ -n "$BIO_MOVEIT_PID" ]]; then
        kill -- "-$BIO_MOVEIT_PID" 2>/dev/null || kill "$BIO_MOVEIT_PID" 2>/dev/null || true
    fi
    [[ -n "$BACKEND_PID" ]] && kill "$BACKEND_PID" 2>/dev/null || true
    [[ -n "$TRAC_MOVEIT_PID" ]] && wait "$TRAC_MOVEIT_PID" 2>/dev/null || true
    [[ -n "$BIO_MOVEIT_PID" ]] && wait "$BIO_MOVEIT_PID" 2>/dev/null || true
    [[ -n "$BACKEND_PID" ]] && wait "$BACKEND_PID" 2>/dev/null || true
    restore_kinematics
}
trap cleanup EXIT INT TERM

for f in "$ROS_SETUP" "$AGX_SETUP" "$VENV_ACTIVATE" "$BACKEND" "$AGENT" "$CORE"; do
    [[ -f "$f" ]] || fatal "Required file not found: $f"
done
status OK "Required NERO V2-6 + frozen V69 files found"

set +u
source "$ROS_SETUP"
[[ -f "$BIO_SETUP" ]] && source "$BIO_SETUP" || status WARN "BioIK workspace setup missing: $BIO_SETUP"
[[ -f "$TRAC_SETUP" ]] && source "$TRAC_SETUP" || status WARN "TRAC-IK workspace setup missing: $TRAC_SETUP"
source "$AGX_SETUP"
source "$VENV_ACTIVATE"
set -u

command -v ros2 >/dev/null 2>&1 || fatal "ros2 CLI unavailable"
ros2 pkg prefix agx_arm_moveit >/dev/null 2>&1 || fatal "agx_arm_moveit package unavailable"
ros2 pkg prefix bio_ik >/dev/null 2>&1 || {
    status INFO "Install BioIK once with: ./install_v63_bioik_humble.sh"
    fatal "BioIK package 'bio_ik' unavailable"
}
status OK "BioIK package available: bio_ik"

TRAC_PKG=""
if ros2 pkg prefix trac_ik_kinematics_plugin >/dev/null 2>&1; then
    TRAC_PKG="trac_ik_kinematics_plugin"
elif ros2 pkg prefix trac_ik_kinematics >/dev/null 2>&1; then
    TRAC_PKG="trac_ik_kinematics"
fi
[[ -n "$TRAC_PKG" ]] || fatal "TRAC-IK plugin unavailable (keep your existing trac_ik_ws install)"
status OK "TRAC-IK package available: $TRAC_PKG"

AGX_MOVEIT_PREFIX="$(ros2 pkg prefix agx_arm_moveit)"
KIN_YAML="${AGX_MOVEIT_PREFIX}/share/agx_arm_moveit/config/kinematics.yaml"
[[ -f "$KIN_YAML" ]] || fatal "AgileX kinematics.yaml not found: $KIN_YAML"
[[ -w "$KIN_YAML" ]] || fatal "kinematics.yaml is not writable: $KIN_YAML"

# Recover from an interrupted V69 launch before taking a fresh backup.
if grep -Eq "NERO_V69_(BIOIK|TRAC_REF)_TEMP" "$KIN_YAML" 2>/dev/null; then
    if [[ -f "$KIN_BACKUP" ]]; then
        cp -f "$KIN_BACKUP" "$KIN_YAML" || fatal "Could not recover stale V69 kinematics.yaml"
        status WARN "Recovered kinematics.yaml left patched by an interrupted V69 run"
    else
        fatal "Stale V69 kinematics marker found but no V69 backup exists"
    fi
fi

cp -f "$KIN_YAML" "$KIN_BACKUP" || fatal "Could not back up kinematics.yaml"
KIN_BACKED_UP=1

write_bio_config(){
cat > "$KIN_YAML" <<EOF
# NERO_V69_BIOIK_TEMP
arm:
  kinematics_solver: bio_ik/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: ${BIO_IK_TIMEOUT}
  kinematics_solver_attempts: 1
  mode: ${BIO_IK_MODE}
  minimal_displacement_weight: ${BIO_MIN_DISP_W}
  avoid_joint_limits_weight: ${BIO_AVOID_LIMIT_W}
  center_joints_weight: ${BIO_CENTER_W}
EOF
}

write_trac_config(){
cat > "$KIN_YAML" <<EOF
# NERO_V69_TRAC_REF_TEMP
arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: ${TRAC_IK_TIMEOUT}
  solve_type: ${TRAC_IK_SOLVE_TYPE}
EOF
}

export NERO_BIO_IK_SOLVER="BioIK"
export NERO_BIO_IK_MODE="$BIO_IK_MODE"
export NERO_BIO_IK_TIMEOUT="$BIO_IK_TIMEOUT"
export NERO_BIO_IK_WEIGHTS="minDisp=${BIO_MIN_DISP_W}, avoidLimit=${BIO_AVOID_LIMIT_W}, center=${BIO_CENTER_W}"
export NERO_TRAC_IK_SOLVER="TRAC-IK"
export NERO_TRAC_IK_SOLVE_TYPE="$TRAC_IK_SOLVE_TYPE"
export NERO_TRAC_IK_TIMEOUT="$TRAC_IK_TIMEOUT"

root_arm_action_ready(){
    timeout 2 ros2 action list 2>/dev/null | grep -qx "/arm_controller/follow_joint_trajectory"
}

root_controller_manager_ready(){
    timeout 2 ros2 service list 2>/dev/null | grep -qx "/controller_manager/list_controllers"
}

ensure_root_arm_controller(){
    for _ in $(seq 1 30); do
        root_arm_action_ready && return 0
        sleep 0.3
    done

    status WARN "Root arm_controller action missing; inspecting controller lifecycle"

    if ! root_controller_manager_ready; then
        status FAIL "Root /controller_manager is unavailable; cannot repair arm_controller"
        return 1
    fi

    CONTROLLERS="$(timeout 4 ros2 control list_controllers -c /controller_manager 2>/dev/null || true)"
    echo "$CONTROLLERS" | sed 's/^/[CONTROLLER] /' >> "$STATUS_LOG"

    if echo "$CONTROLLERS" | grep -Eq '^arm_controller[[:space:]]'; then
        if ! echo "$CONTROLLERS" | grep -Eq '^arm_controller[[:space:]].*[[:space:]]active([[:space:]]|$)'; then
            status WARN "arm_controller exists but is not active; activating it"
            timeout 8 ros2 control set_controller_state arm_controller active -c /controller_manager \
                >>"$STATUS_LOG" 2>&1 || true
        fi
    else
        status WARN "arm_controller is not loaded; loading + activating it"
        timeout 10 ros2 control load_controller arm_controller --set-state active -c /controller_manager \
            >>"$STATUS_LOG" 2>&1 || true
    fi

    for _ in $(seq 1 30); do
        if root_arm_action_ready; then
            status OK "Root arm_controller FollowJointTrajectory action is READY"
            return 0
        fi
        sleep 0.3
    done

    status FAIL "Could not restore /arm_controller/follow_joint_trajectory"
    timeout 4 ros2 control list_controllers -c /controller_manager 2>/dev/null \
        | sed 's/^/[CONTROLLER FINAL] /' >> "$STATUS_LOG" || true
    return 1
}

cd "$NERO_WS"
status INFO "Starting NERO V2-6 backend + Tk GUI + MuJoCo"
python3 -u "$BACKEND" >"$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!
status OK "Backend started PID=$BACKEND_PID"

# Wait for physical feedback bridge before MoveIt.
for i in $(seq 1 100); do
    kill -0 "$BACKEND_PID" 2>/dev/null || fatal "Backend exited during startup"
    if timeout 1 ros2 topic list 2>/dev/null | grep -qx "/feedback/joint_states"; then
        status OK "ROS topic READY: /feedback/joint_states"
        break
    fi
    [[ "$i" -eq 100 ]] && fatal "Backend bridge timeout: /feedback/joint_states"
    sleep 0.4
done

# ------------------------------------------------------------
# Primary root MoveIt: BioIK
# IMPORTANT V69: DO NOT pass control:=false here. AgileX demo.launch.py
# must start ros2_control + arm_controller just like the proven V66 launcher.
# ------------------------------------------------------------
write_bio_config
status OK "Temporarily configured root MoveIt for BioIK"
status INFO "BioIK mode=$BIO_IK_MODE timeout=${BIO_IK_TIMEOUT}s weights=[$NERO_BIO_IK_WEIGHTS]"

setsid ros2 launch agx_arm_moveit demo.launch.py \
    arm_type:=nero \
    effector_type:=agx_gripper \
    follow:=true \
    feedback_topic:=feedback/joint_states \
    control_topic:=control/joint_states \
    tcp_offset:='[0.1553, 0.0, -0.0215, -1.5708, 0.0, -1.5708]' \
    use_rviz:="${USE_RVIZ}" \
    >"$BIO_MOVEIT_LOG" 2>&1 &
BIO_MOVEIT_PID=$!
status OK "Primary BioIK MoveIt launch PID=$BIO_MOVEIT_PID"

BIO_READY=0
for i in $(seq 1 150); do
    kill -0 "$BIO_MOVEIT_PID" 2>/dev/null || fatal "Primary BioIK MoveIt exited"
    NODES="$(timeout 1 ros2 node list 2>/dev/null || true)"
    SERVICES="$(timeout 1 ros2 service list 2>/dev/null || true)"
    if echo "$NODES" | grep -qx "/move_group" && echo "$SERVICES" | grep -qx "/compute_ik"; then
        SOLVER="$(timeout 2 ros2 param get /move_group robot_description_kinematics.arm.kinematics_solver 2>/dev/null || true)"
        if echo "$SOLVER" | grep -q "bio_ik/BioIKKinematicsPlugin"; then
            status OK "Primary /move_group active IK solver verified: BioIK"
            BIO_READY=1
            break
        fi
    fi
    sleep 0.4
done
[[ "$BIO_READY" -eq 1 ]] || fatal "BioIK MoveIt did not become ready/verified"

# V69: root execution is a hard prerequisite before starting the reference instance.
ensure_root_arm_controller || {
    tail -n 50 "$BIO_MOVEIT_LOG" | sed 's/^/[BIO MOVEIT] /' >> "$STATUS_LOG" || true
    fatal "Primary MoveIt is up, but root arm_controller could not be made ACTIVE"
}

# ------------------------------------------------------------
# Secondary namespaced MoveIt: TRAC-IK reference/fallback only
# Keep V66 launch semantics here too; the namespace isolates its controller stack.
# The namespace prevents its controllers/control topics from owning the
# physical root arm. It follows the same global /feedback/joint_states.
# ------------------------------------------------------------
write_trac_config
status OK "Temporarily configured second MoveIt launch for TRAC-IK"

setsid ros2 launch agx_arm_moveit demo.launch.py \
    namespace:=trac_ref \
    arm_type:=nero \
    effector_type:=agx_gripper \
    follow:=true \
    feedback_topic:=/feedback/joint_states \
    control_topic:=control/joint_states \
    tcp_offset:='[0.1553, 0.0, -0.0215, -1.5708, 0.0, -1.5708]' \
    use_rviz:=false \
    >"$TRAC_MOVEIT_LOG" 2>&1 &
TRAC_MOVEIT_PID=$!
status OK "TRAC reference MoveIt launch PID=$TRAC_MOVEIT_PID"

TRAC_READY=0
for i in $(seq 1 150); do
    kill -0 "$TRAC_MOVEIT_PID" 2>/dev/null || fatal "TRAC reference MoveIt exited"
    NODES="$(timeout 1 ros2 node list 2>/dev/null || true)"
    SERVICES="$(timeout 1 ros2 service list 2>/dev/null || true)"
    if echo "$NODES" | grep -qx "/trac_ref/move_group" && echo "$SERVICES" | grep -qx "/trac_ref/compute_ik"; then
        SOLVER="$(timeout 2 ros2 param get /trac_ref/move_group robot_description_kinematics.arm.kinematics_solver 2>/dev/null || true)"
        if echo "$SOLVER" | grep -q "TRAC_IKKinematicsPlugin"; then
            status OK "Reference /trac_ref/move_group active IK solver verified: TRAC-IK"
            TRAC_READY=1
            break
        fi
    fi
    sleep 0.4
done
[[ "$TRAC_READY" -eq 1 ]] || fatal "TRAC reference MoveIt did not become ready/verified"

# The reference instance must not deactivate/steal the physical root controller.
ensure_root_arm_controller || fatal "TRAC reference startup disturbed root arm_controller"

# Both processes have loaded their parameters; restore the installed file now.
restore_kinematics

# Root execution interfaces required by the GUI.
READY=0
for i in $(seq 1 150); do
    ACTIONS="$(timeout 1 ros2 action list 2>/dev/null || true)"
    SERVICES="$(timeout 1 ros2 service list 2>/dev/null || true)"
    TOPICS="$(timeout 1 ros2 topic list 2>/dev/null || true)"
    if echo "$ACTIONS" | grep -qx "/move_action" \
       && echo "$ACTIONS" | grep -qx "/arm_controller/follow_joint_trajectory" \
       && echo "$SERVICES" | grep -qx "/compute_cartesian_path" \
       && echo "$SERVICES" | grep -qx "/compute_ik" \
       && echo "$SERVICES" | grep -qx "/trac_ref/compute_ik" \
       && echo "$TOPICS" | grep -qx "/planning_scene"; then
        READY=1
        status READY "SYSTEM READY: BioIK primary + TRAC reference + MuJoCo + root controllers"
        break
    fi
    sleep 0.4
done

if [[ "$READY" -ne 1 ]]; then
    tail -n 50 "$BIO_MOVEIT_LOG" | sed 's/^/[BIO MOVEIT] /' >> "$STATUS_LOG" || true
    tail -n 50 "$TRAC_MOVEIT_LOG" | sed 's/^/[TRAC REF] /' >> "$STATUS_LOG" || true
    fatal "NERO V2-6 final readiness failed; GUI execution is intentionally blocked"
fi

status INFO "Close the NERO V2-6 GUI to stop Agent UI, both MoveIt instances, and MuJoCo"
wait "$BACKEND_PID"
