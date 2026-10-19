#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# NERO V2-7.4 — Agent Orchestration Hardening
#
# Source runtime (already validated locally):
#   ~/mujoco_ws/nero/NERO_V2_7_3_Late_Settle_Task_Summary
#
# Generated runtime:
#   ~/mujoco_ws/nero/NERO_V2_7_4_Agent_Orchestration_Hardening
#
# This wrapper performs strict source-file checks, builds V2-7.4, runs the
# upgrader's Python/bash/hash/path validation, then starts ONLY the generated
# V2-7.4 launcher.
# ============================================================

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NERO_ROOT="${NERO_ROOT:-$HOME/mujoco_ws/nero}"
SRC_DIR="${NERO_V273_DIR:-$NERO_ROOT/NERO_V2_7_3_Late_Settle_Task_Summary}"
OUT_DIR="${NERO_V274_DIR:-$NERO_ROOT/NERO_V2_7_4_Agent_Orchestration_Hardening}"
UPGRADER="$HERE/tools/upgrade_v2_7_3_to_v2_7_4.py"

fail(){ echo "[FAIL] $*" >&2; exit 1; }
info(){ echo "[INFO] $*"; }
ok(){ echo "[OK] $*"; }

[[ -f "$UPGRADER" ]] || fail "V2-7.4 upgrader missing: $UPGRADER"

required=(
  NERO_V2_7_1_Unified_Backend.py
  NERO_V2_7_1_Agent.py
  NERO_V69_ReachableZone_BioIK_Planner_Core.py
  start_nero_v2_7_3.sh
)
for f in "${required[@]}"; do
  [[ -f "$SRC_DIR/$f" ]] || fail "Validated V2-7.3 source file missing: $SRC_DIR/$f"
done

info "Validated source runtime: $SRC_DIR"
info "Generating V2-7.4 runtime: $OUT_DIR"
python3 "$UPGRADER" "$SRC_DIR" --output-dir "$OUT_DIR"

LAUNCHER="$OUT_DIR/start_nero_v2_7_4.sh"
BACKEND="$OUT_DIR/NERO_V2_7_4_Unified_Backend.py"
AGENT="$OUT_DIR/NERO_V2_7_4_Agent.py"
CORE="$OUT_DIR/NERO_V69_ReachableZone_BioIK_Planner_Core.py"
SUMS="$OUT_DIR/SHA256SUMS.txt"

for f in "$LAUNCHER" "$BACKEND" "$AGENT" "$CORE" "$SUMS"; do
  [[ -f "$f" ]] || fail "Generated V2-7.4 runtime file missing: $f"
done

bash -n "$LAUNCHER" || fail "Generated launcher failed bash -n: $LAUNCHER"
python3 -m py_compile "$BACKEND" "$AGENT" || fail "Generated Python entry file failed py_compile"

EXPECTED_V69="3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7"
ACTUAL_V69="$(sha256sum "$CORE" | awk '{print $1}')"
[[ "$ACTUAL_V69" == "$EXPECTED_V69" ]] || fail "Frozen V69 SHA mismatch: $ACTUAL_V69"

if grep -q 'NERO_V2_7_1_Unified_Backend.py\|NERO_V2_7_1_Agent.py' "$LAUNCHER"; then
  fail "Generated launcher still contains stale V2-7.1 entry filenames"
fi
grep -q 'NERO_V2_7_4_Unified_Backend.py' "$LAUNCHER" || fail "Launcher missing V2-7.4 backend path"
grep -q 'NERO_V2_7_4_Agent.py' "$LAUNCHER" || fail "Launcher missing V2-7.4 Agent path"

ochmod +x "$LAUNCHER"
ok "V2-7.4 build + path + syntax + frozen-core checks passed"
echo "[INFO] Generated file contract:"
echo "       $BACKEND"
echo "       $AGENT"
echo "       $CORE"
echo "       $LAUNCHER"
echo
info "Launching generated V2-7.4 runtime..."
cd "$OUT_DIR"
exec "$LAUNCHER"
