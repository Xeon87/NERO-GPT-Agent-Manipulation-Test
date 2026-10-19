#!/usr/bin/env python3
"""Run the V2-7 local upgrade payload from this experimental branch."""
from pathlib import Path
import sys

payload = Path(__file__).with_name("upgrade_v2_6_egg_to_v2_7.py")
text = payload.read_text(encoding="utf-8")
# Repair a packaging-only quote typo in the payload's generated README text.
text = text.replace(
    '        """# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    '        "# NERO V2-7 — Constraint-Aware Grasp Planning\\n\\n"\n',
    1,
)
code = compile(text, str(payload), "exec")
ns = {
    "__name__": "__main__",
    "__file__": str(payload),
    "__package__": None,
}
exec(code, ns, ns)
