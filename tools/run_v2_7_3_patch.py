#!/usr/bin/env python3
"""Compatibility shim for the actual generated V2-7.2 runtime filenames.

V2-7.2 is an incremental hardening layer over V2-7.1 and intentionally keeps
its runtime Python filenames as NERO_V2_7_1_*.  The generic V2-7.3 upgrader
contains the patch logic; this shim points it at the files that really exist on
the user's Ubuntu runtime.
"""
import upgrade_v2_7_2_to_v2_7_3 as impl

impl.BACKEND = "NERO_V2_7_1_Unified_Backend.py"
impl.AGENT = "NERO_V2_7_1_Agent.py"
impl.CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

if __name__ == "__main__":
    impl.main()
