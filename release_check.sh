#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

echo "[1/5] Python syntax"
python3 -m py_compile \
  NERO_V2_6_Agent.py \
  NERO_V2_6_Unified_Backend.py \
  NERO_V69_ReachableZone_BioIK_Planner_Core.py

echo "[2/5] Launcher syntax"
bash -n start_nero_v2_6.sh

echo "[3/5] Secret-pattern sanity check"
if grep -RInE --exclude='release_check.sh' --exclude='README.md' --exclude='QUICKSTART.md' \
  'sk-[A-Za-z0-9_-]{16,}|OPENAI_API_KEY=[A-Za-z0-9_-]{16,}' . ; then
  echo "Potential committed secret found" >&2
  exit 2
fi

echo "[4/5] Required public docs"
for f in README.md QUICKSTART.md ARCHITECTURE.md YOUTUBE_DEMO.md CHANGELOG.md THIRD_PARTY_NOTICES.md LICENSE.md; do
  test -s "$f" || { echo "Missing: $f" >&2; exit 3; }
done

echo "[5/5] Frozen V69 SHA256"
sha256sum NERO_V69_ReachableZone_BioIK_Planner_Core.py

echo "PUBLIC_RELEASE_STATIC_CHECKS_OK"
