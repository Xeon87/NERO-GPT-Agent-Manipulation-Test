#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME="$HOME/mujoco_ws/nero/NERO_V2_7_2_Hold_Continuity_Wrist_Reorientation"
LAUNCHER="$RUNTIME/start_nero_v2_7_2.sh"
BACKEND="$RUNTIME/NERO_V2_7_1_Unified_Backend.py"
AGENT="$RUNTIME/NERO_V2_7_1_Agent.py"
CORE="$RUNTIME/NERO_V69_ReachableZone_BioIK_Planner_Core.py"
PATCH="$HERE/tools/apply_v2_11_orient_cartesian.py"

printf '%s\n' \
  '============================================================' \
  ' NERO V2-11 — V2-10 Base + Cartesian orient_for_slot Transfer' \
  '============================================================'

echo "[INFO] Runtime: $RUNTIME"

for f in "$BACKEND" "$AGENT" "$CORE" "$LAUNCHER" "$PATCH"; do
  if [[ ! -f "$f" ]]; then
    echo "ERROR: required file missing: $f"
    exit 2
  fi
done

if grep -q 'V2-11 ORIENT CARTESIAN TRANSFER' "$BACKEND"; then
  echo '[OK] V2-11 orient_for_slot Cartesian patch is already present.'
else
  if ! grep -q 'V2-10 ORIENT TRANSFER' "$BACKEND"; then
    echo 'ERROR: V2-11 expects the V2-10 runtime that you just tested.'
    echo '       V2-10 orient_for_slot marker was not found; refusing to modify any other baseline.'
    exit 3
  fi

  BACKUP="$RUNTIME/.v2_10_backend_before_v2_11.py"
  if [[ ! -f "$BACKUP" ]]; then
    cp -p "$BACKEND" "$BACKUP"
    echo "[OK] Saved V2-10 backend backup: $BACKUP"
  fi

  echo '[INFO] Applying ONLY V2-11 orient_for_slot transport patch...'
  python3 "$PATCH" "$RUNTIME"
fi

chmod +x "$LAUNCHER"

echo
echo '[INFO] Launching through unchanged V2-7.2 runtime launcher:'
echo "       $LAUNCHER"
echo '[INFO] V2-11 delta vs V2-10: orient_for_slot transport only'
echo '[INFO] Transport = one continuous Cartesian path: vertical clearance + high-Z XY'
echo

exec "$LAUNCHER"
