#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# NERO V2-7.1 — Placement-Aware Egg Handling
# One-shot installer/launcher from the already-validated local V2-7 runtime.
#
# Default source:
#   ~/mujoco_ws/nero/NERO_V2_7_Constraint_Aware_Grasp_Planning
#
# Generated runtime:
#   ~/mujoco_ws/nero/NERO_V2_7_1_Placement_Aware_Egg_Handling
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NERO_ROOT="${NERO_ROOT:-$HOME/mujoco_ws/nero}"
SRC_DIR="${NERO_V27_DIR:-$NERO_ROOT/NERO_V2_7_Constraint_Aware_Grasp_Planning}"
OUT_DIR="${NERO_V271_DIR:-$NERO_ROOT/NERO_V2_7_1_Placement_Aware_Egg_Handling}"
UPGRADER="$SCRIPT_DIR/tools/upgrade_v2_7_to_v2_7_1.py"

fail(){ echo "[FAIL] $*" >&2; exit 1; }
info(){ echo "[INFO] $*"; }
ok(){ echo "[OK] $*"; }

[[ -f "$UPGRADER" ]] || fail "Bundled V2-7.1 upgrader not found: $UPGRADER"

required=(
  NERO_V2_7_Unified_Backend.py
  NERO_V2_7_Agent.py
  NERO_V69_ReachableZone_BioIK_Planner_Core.py
  start_nero_v2_7.sh
)
for f in "${required[@]}"; do
  [[ -f "$SRC_DIR/$f" ]] || fail "Expected validated V2-7 runtime file missing: $SRC_DIR/$f"
done

info "Source V2-7 runtime: $SRC_DIR"
info "Building V2-7.1 runtime: $OUT_DIR"
python3 "$UPGRADER" "$SRC_DIR" --output-dir "$OUT_DIR"

LAUNCHER="$OUT_DIR/start_nero_v2_7_1.sh"
[[ -f "$LAUNCHER" ]] || fail "Generated V2-7.1 launcher missing: $LAUNCHER"
chmod +x "$LAUNCHER"

ok "V2-7.1 runtime built successfully"
echo "[INFO] Recommended first test:"
echo "       Pick up egg_3 and place it into the first empty egg-holder slot."
echo
info "Launching NERO V2-7.1..."
cd "$OUT_DIR"
exec "$LAUNCHER"
