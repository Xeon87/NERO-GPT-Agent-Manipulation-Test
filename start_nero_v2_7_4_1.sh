#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# NERO V2-7.4.1 — GUI Startup Hotfix
#
# Validated source runtime:
#   ~/mujoco_ws/nero/NERO_V2_7_3_Late_Settle_Task_Summary
#
# Generated runtime:
#   ~/mujoco_ws/nero/NERO_V2_7_4_1_GUI_Startup_Hotfix
#
# This wrapper generates the hotfix runtime, validates exact paths/names,
# Python syntax, Bash syntax, and frozen V69 SHA, then starts ONLY V2-7.4.1.
# ============================================================

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NERO_ROOT="${NERO_ROOT:-$HOME/mujoco_ws/nero}"
SRC_DIR="${NERO_V273_DIR:-$NERO_ROOT/NERO_V2_7_3_Late_Settle_Task_Summary}"
OUT_DIR="${NERO_V2741_DIR:-$NERO_ROOT/NERO_V2_7_4_1_GUI_Startup_Hotfix}"
UPGRADER="$HERE/tools/upgrade_v2_7_3_to_v2_7_4_1.py"

fail(){ echo "[FAIL] $*" >&2; exit 1; }
info(){ echo "[INFO] $*"; }
ok(){ echo "[OK] $*"; }

[[ -f "$UPGRADER" ]] || fail "V2-7.4.1 upgrader missing: $UPGRADER"

required_source=(
  NERO_V2_7_1_Unified_Backend.py
  NERO_V2_7_1_Agent.py
  NERO_V69_ReachableZone_BioIK_Planner_Core.py
  start_nero_v2_7_3.sh
)
for f in "${required_source[@]}"; do
  [[ -f "$SRC_DIR/$f" ]] || fail "Validated V2-7.3 source file missing: $SRC_DIR/$f"
done

info "Validated source runtime: $SRC_DIR"
info "Generating V2-7.4.1 runtime: $OUT_DIR"
python3 "$UPGRADER" "$SRC_DIR" --output-dir "$OUT_DIR"

LAUNCHER="$OUT_DIR/start_nero_v2_7_4_1.sh"
BACKEND="$OUT_DIR/NERO_V2_7_4_1_Unified_Backend.py"
AGENT="$OUT_DIR/NERO_V2_7_4_1_Agent.py"
CORE="$OUT_DIR/NERO_V69_ReachableZone_BioIK_Planner_Core.py"
SUMS="$OUT_DIR/SHA256SUMS.txt"
PATH_CHECK="$OUT_DIR/PATH_CHECK.txt"

for f in "$LAUNCHER" "$BACKEND" "$AGENT" "$CORE" "$SUMS" "$PATH_CHECK"; do
  [[ -f "$f" ]] || fail "Generated V2-7.4.1 runtime file missing: $f"
done

bash -n "$LAUNCHER" || fail "Generated launcher failed bash -n: $LAUNCHER"
python3 -m py_compile "$BACKEND" "$AGENT" || fail "Generated Python entry file failed py_compile"

EXPECTED_V69="3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
ACTUAL_V69="$(sha256sum "$CORE" | awk '{print $1}')"
[[ "$ACTUAL_V69" == "$EXPECTED_V69" ]] || fail "Frozen V69 SHA mismatch: $ACTUAL_V69"

# Strict filename/path contract.
for stale in \
  NERO_V2_7_1_Unified_Backend.py \
  NERO_V2_7_1_Agent.py \
  NERO_V2_7_4_Unified_Backend.py \
  NERO_V2_7_4_Agent.py; do
  if grep -q "$stale" "$LAUNCHER"; then
    fail "Generated launcher contains stale entry filename: $stale"
  fi
done

grep -q 'NERO_V2_7_4_1_Unified_Backend.py' "$LAUNCHER" || fail "Launcher missing V2-7.4.1 backend path"
grep -q 'NERO_V2_7_4_1_Agent.py' "$LAUNCHER" || fail "Launcher missing V2-7.4.1 Agent path"
grep -q 'Post-readiness GUI/backend liveness check (3.0s)' "$LAUNCHER" || fail "Launcher missing GUI/backend liveness gate"
grep -q '_v2741_bold_tag_ready' "$BACKEND" || fail "Backend missing lazy bold-log marker"
grep -q 'LLM ORCHESTRATOR AUTO-CLAMP' "$AGENT" || fail "Agent missing orchestration auto-clamp marker"
grep -q 'USER_ABORTED' "$AGENT" || fail "Agent missing user-stop summary marker"

chmod +x "$LAUNCHER"
ok "V2-7.4.1 build + path + syntax + frozen-core checks passed"
echo "[INFO] Exact generated runtime files:"
echo "       $BACKEND"
echo "       $AGENT"
echo "       $CORE"
echo "       $LAUNCHER"
echo
info "Launching generated V2-7.4.1 runtime..."
cd "$OUT_DIR"
exec "$LAUNCHER"
