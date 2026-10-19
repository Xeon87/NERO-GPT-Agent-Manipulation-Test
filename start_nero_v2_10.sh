#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo " NERO V2-10 — V2-9 Base + orient_for_slot Staged Transfer"
echo "============================================================"

# Recreate the exact validated V2-7.2 runtime using the original V2-7.2 path.
python3 "$HERE/tools/run_v2_7_2_upgrader.py"

RUNTIME="$HOME/mujoco_ws/nero/NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation"
LAUNCHER="$RUNTIME/start_nero_v2_7_2.sh"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "ERROR: generated V2-7.2 launcher missing: $LAUNCHER"
  exit 3
fi

# Preserve V2-9 presentation + high-level summary exactly.
python3 "$HERE/tools/apply_v2_9_log_summary.py" "$RUNTIME"

# V2-10 ONLY behavior change: orient_for_slot Step A staged transfer.
python3 "$HERE/tools/apply_v2_10_orient_for_slot.py" "$RUNTIME"

chmod +x "$LAUNCHER"

echo
echo "[INFO] Starting V2-10 on the exact V2-7.2 runtime path..."
echo "[INFO] Runtime: $RUNTIME"
echo "[INFO] Runtime launcher (unchanged V2-7.2): $LAUNCHER"
echo "[INFO] Only runtime behavior delta vs V2-9: orient_for_slot staged safe transfer"
echo

exec "$LAUNCHER"
