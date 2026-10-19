#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "============================================================"
echo " NERO V2-7.2 — Hold Continuity + Bounded Wrist Reorientation"
echo "============================================================"

python3 "$HERE/tools/run_v2_7_2_upgrader.py"

RUNTIME="$HOME/mujoco_ws/nero/NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation"
LAUNCHER="$RUNTIME/start_nero_v2_7_2.sh"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "ERROR: generated launcher missing: $LAUNCHER"
  exit 3
fi

chmod +x "$LAUNCHER"

echo
echo "[INFO] Starting generated V2-7.2 runtime..."
echo "[INFO] Runtime: $RUNTIME"
echo

exec "$LAUNCHER"
