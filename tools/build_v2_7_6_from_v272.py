#!/usr/bin/env python3
from pathlib import Path
import argparse, hashlib, py_compile, shutil, subprocess, re

V69_SHA='3afc191590aa7a6959f23fac524f7901145fe5d838532d41dd25ea47c462f4b7'


def sha(p):
    h=hashlib.sha256(); h.update(Path(p).read_bytes()); return h.hexdigest()


def replace_once(s,a,b,label):
    n=s.count(a)
    if n!=1: raise RuntimeError(f'{label}: expected 1 anchor, found {n}')
    return s.replace(a,b,1)


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('v272'); ap.add_argument('v275'); ap.add_argument('--output-dir',required=True); a=ap.parse_args()
    v272=Path(a.v272); v275=Path(a.v275); out=Path(a.output_dir)
    if out.exists(): shutil.rmtree(out)
    out.mkdir(parents=True)

    # IMPORTANT: preserve exact V2-7.2 runtime module names to keep multiprocessing spawn/import behavior unchanged.
    backend_src=v272/'NERO_V2_7_1_Unified_Backend.py'
    agent_src_v275=v275/'NERO_V2_7_5_Agent.py'
    launcher_src=v272/'start_nero_v2_7_2.sh'
    core_src=v272/'NERO_V69_ReachableZone_BioIK_Planner_Core.py'
    for p in (backend_src,agent_src_v275,launcher_src,core_src):
        if not p.exists(): raise FileNotFoundError(p)

    backend=backend_src.read_text(encoding='utf-8')
    backend_original=backend
    old='''    def append_gui_log(text):\n        global_log.insert("end", str(text).rstrip() + "\\n")\n        global_log.see("end")\n'''
    new='''    def append_gui_log(text):\n        _msg = str(text).rstrip()\n        # V2-7.6 display-only styling. No startup, MuJoCo, ROS or motion state is touched.\n        if not getattr(append_gui_log, "_v276_tags_ready", False):\n            try:\n                global_log.tag_configure("llm_message_bold", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_success", foreground="#138A36", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_warning", foreground="#A66B00", font=("Arial", 10, "bold"))\n                global_log.tag_configure("task_summary_failure", foreground="#C62828", font=("Arial", 10, "bold"))\n            except Exception:\n                pass\n            append_gui_log._v276_tags_ready = True\n        _tag = None\n        if _msg.startswith("[TASK SUMMARY GREEN]"):\n            _tag = "task_summary_success"\n        elif _msg.startswith("[TASK SUMMARY YELLOW]"):\n            _tag = "task_summary_warning"\n        elif _msg.startswith("[TASK SUMMARY RED]"):\n            _tag = "task_summary_failure"\n        elif "[LLM " in _msg:\n            _tag = "llm_message_bold"\n        if _tag:\n            global_log.insert("end", _msg + "\\n", _tag)\n        else:\n            global_log.insert("end", _msg + "\\n")\n        global_log.see("end")\n'''
    backend=replace_once(backend,old,new,'append_gui_log')
    (out/'NERO_V2_7_1_Unified_Backend.py').write_text(backend,encoding='utf-8')

    # Reuse the already unit-tested V2-7.5 orchestration Agent, but restore the exact filename imported by V2-7.2 backend.
    agent=agent_src_v275.read_text(encoding='utf-8')
    agent=agent.replace('PROJECT_VERSION = "2.7.5"','PROJECT_VERSION = "2.7.6"',1)
    agent=agent.replace('# NERO V2-7.5: Agent orchestration hardening on validated V2-7.2 motion/startup baseline.','# NERO V2-7.6: Agent orchestration fixes on exact V2-7.2 backend/module baseline.',1)
    (out/'NERO_V2_7_1_Agent.py').write_text(agent,encoding='utf-8')

    shutil.copy2(core_src,out/core_src.name)

    # Launcher: byte-identical V2-7.2 startup semantics. Only user-visible version label changes.
    launcher=launcher_src.read_text(encoding='utf-8').replace('NERO V2-7.2','NERO V2-7.6')
    (out/'start_nero_v2_7_6.sh').write_text(launcher,encoding='utf-8')
    (out/'start_nero_v2_7_6.sh').chmod(0o755)

    (out/'README_V2_7_6.md').write_text('''# NERO V2-7.6\n\nBaseline: validated V2-7.2 runtime.\n\nChanges only:\n1. Agent verified approach -> deterministic immediate clamp.\n2. User Stop -> guaranteed high-level summary.\n3. LLM log messages bold; task summary color/bold.\n\nCritical compatibility decision: runtime backend/Agent module filenames remain the exact V2-7.2 names so Python multiprocessing spawn and MuJoCo child import behavior are unchanged.\n''',encoding='utf-8')
    (out/'PATH_CHECK.txt').write_text('''NERO V2-7.6 runtime contract\n./NERO_V2_7_1_Unified_Backend.py\n./NERO_V2_7_1_Agent.py\n./NERO_V69_ReachableZone_BioIK_Planner_Core.py\n./start_nero_v2_7_6.sh\n\nThe legacy-looking Python filenames are intentional: they preserve the exact proven V2-7.2 multiprocessing/MuJoCo import path.\n''',encoding='utf-8')

    # Syntax + frozen core.
    for p in (out/'NERO_V2_7_1_Unified_Backend.py',out/'NERO_V2_7_1_Agent.py',out/core_src.name): py_compile.compile(str(p),doraise=True)
    subprocess.run(['bash','-n',str(out/'start_nero_v2_7_6.sh')],check=True)
    if sha(out/core_src.name)!=V69_SHA: raise RuntimeError('V69 SHA mismatch')

    # Prove MuJoCo child and main/process startup functions are text-identical to V2-7.2.
    def fn_block(text,name,next_name):
        a=text.index(f'def {name}('); b=text.index(f'\ndef {next_name}(',a) if next_name else len(text); return text[a:b]
    if fn_block(backend_original,'run_sim','run_gui') != fn_block(backend,'run_sim','run_gui'):
        raise RuntimeError('run_sim changed from V2-7.2')
    # main contains GUI wiring including append_gui_log indirectly elsewhere; verify the exact process creation segment instead.
    seg0=backend_original[backend_original.index('    sim_process = mp.Process('):backend_original.index('    try:\n        run_gui(',backend_original.index('    sim_process = mp.Process('))]
    seg1=backend[backend.index('    sim_process = mp.Process('):backend.index('    try:\n        run_gui(',backend.index('    sim_process = mp.Process('))]
    if seg0!=seg1: raise RuntimeError('MuJoCo mp.Process startup segment changed')

    sums=[]
    for n in ['NERO_V2_7_1_Unified_Backend.py','NERO_V2_7_1_Agent.py','NERO_V69_ReachableZone_BioIK_Planner_Core.py','start_nero_v2_7_6.sh']:
        sums.append(f'{sha(out/n)}  {n}')
    (out/'SHA256SUMS.txt').write_text('\n'.join(sums)+'\n',encoding='utf-8')
    print('[OK] V2-7.6 built from V2-7.2 backend/module paths')
    print('[OK] run_sim text identical to V2-7.2')
    print('[OK] MuJoCo mp.Process startup segment identical to V2-7.2')
    print('[OK] V69 frozen')

if __name__=='__main__': main()
