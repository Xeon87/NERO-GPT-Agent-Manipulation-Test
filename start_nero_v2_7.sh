#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# NERO V2-7 — Constraint-Aware Grasp Planning
# One-shot launcher for the Egg Task Semantic Integration line.
#
# This package intentionally keeps the validated V69 motion core frozen.
# It upgrades the already-tested local V2-6 Egg Semantic package into a
# dedicated V2-7 runtime folder, then launches that generated runtime.
#
# Default expected local source:
#   ~/mujoco_ws/nero/NERO_V2_6_Egg_Task_Semantic_Integration
# Override with:
#   NERO_V26_EGG_DIR=/path/to/source ./start_nero_v2_7.sh
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NERO_ROOT="${NERO_ROOT:-$HOME/mujoco_ws/nero}"
SRC_DIR="${NERO_V26_EGG_DIR:-$NERO_ROOT/NERO_V2_6_Egg_Task_Semantic_Integration}"
OUT_DIR="${NERO_V27_DIR:-$NERO_ROOT/NERO_V2_7_Constraint_Aware_Grasp_Planning}"
UPGRADER="${SCRIPT_DIR}/tools/run_v2_7_upgrader.py"

fail(){ echo "[FAIL] $*" >&2; exit 1; }
info(){ echo "[INFO] $*"; }
ok(){ echo "[OK] $*"; }

[[ -f "$UPGRADER" ]] || fail "Bundled V2-7 upgrader not found: $UPGRADER"

required=(
  NERO_V2_6_Unified_Backend.py
  NERO_V2_6_Agent.py
  NERO_V69_ReachableZone_BioIK_Planner_Core.py
  start_nero_v2_6.sh
)

for f in "${required[@]}"; do
  [[ -f "$SRC_DIR/$f" ]] || fail "Expected V2-6 Egg Semantic source file missing: $SRC_DIR/$f"
done

info "Source V2-6 Egg Semantic package: $SRC_DIR"
info "Generating V2-7 runtime: $OUT_DIR"
python3 "$UPGRADER" "$SRC_DIR" --output-dir "$OUT_DIR"

LAUNCHER="$OUT_DIR/start_nero_v2_7.sh"
[[ -f "$LAUNCHER" ]] || fail "Generated V2-7 launcher missing: $LAUNCHER"
chmod +x "$LAUNCHER"

ok "V2-7 runtime generated successfully"
echo "[INFO] First recommended test task:"
echo "       Pick up egg_1 and place it into the first empty egg-holder slot."
echo
info "Launching NERO V2-7..."
cd "$OUT_DIR"
exec "$LAUNCHER"
