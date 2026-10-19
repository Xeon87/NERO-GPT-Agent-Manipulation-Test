#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo " NERO V2-9 — V2-7.2 Base + Log Styling + High-Level Summary"
echo "============================================================"

# 1) Recreate the validated V2-7.2 runtime exactly the same way as the
#    original V2-7.2 top-level launcher.
python3 "$HERE/tools/run_v2_7_2_upgrader.py"

# IMPORTANT: keep the exact V2-7.2 runtime path and second-layer launcher.
RUNTIME="$HOME/mujoco_ws/nero/NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation"
LAUNCHER="$RUNTIME/start_nero_v2_7_2.sh"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "ERROR: generated V2-7.2 launcher missing: $LAUNCHER"
  exit 3
fi

# 2) Apply ONLY the V2-9 presentation + task-summary patch in place.
python3 "$HERE/tools/apply_v2_9_log_summary.py" "$RUNTIME"

chmod +x "$LAUNCHER"

echo
echo "[INFO] Starting V2-9 on the exact V2-7.2 runtime path..."
echo "[INFO] Runtime: $RUNTIME"
echo "[INFO] Runtime launcher (unchanged V2-7.2): $LAUNCHER"
echo

exec "$LAUNCHER"
