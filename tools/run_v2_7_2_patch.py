#!/usr/bin/env python3
"""Compatibility shim for the actual generated V2-7.1 runtime filenames."""
import upgrade_v2_7_1_to_v2_7_2 as impl

impl.BACKEND = "NERO_V2_7_1_Unified_Backend.py"
impl.AGENT = "NERO_V2_7_1_Agent.py"
impl.CORE = "NERO_V69_ReachableZone_BioIK_Planner_Core.py"

if __name__ == "__main__":
    impl.main()
