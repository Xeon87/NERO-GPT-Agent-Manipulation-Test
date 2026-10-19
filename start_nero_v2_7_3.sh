#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo " NERO V2-7.3 — Late-Settle Verification + Task-End Summary"
echo "============================================================"

python3 "$HERE/tools/run_v2_7_3_upgrader.py"

RUNTIME="$HOME/mujoco_ws/nero/NERO_V2_7_3_Late_Settle_Task_Summary"
LAUNCHER="$RUNTIME/start_nero_v2_7_3.sh"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "ERROR: generated launcher missing: $LAUNCHER"
  exit 3
fi

chmod +x "$LAUNCHER"

echo
echo "[INFO] Starting generated V2-7.3 runtime..."
echo "[INFO] Runtime: $RUNTIME"
echo

exec "$LAUNCHER"
