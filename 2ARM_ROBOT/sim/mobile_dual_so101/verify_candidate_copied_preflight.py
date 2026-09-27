#!/usr/bin/env python3
"""Copied five-phase check using the same candidate planner as live SIM."""
import argparse
import json
from pathlib import Path
from dynamic_preflight import full_state_preflight
from run_live_connection_candidate import LiveConnectionTeacher


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parent
    parser.add_argument('--config',type=Path,default=root/'config/runtime_contact_candidate.json')
    parser.add_argument('--staging',type=Path,default=root/'config/runtime_staging_reference.json')
    parser.add_argument('--donor',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    # Reserve output before running so an old result cannot be overwritten.
    with a.output.open('x') as stream:
        config=json.loads(a.config.read_text());staging=json.loads(a.staging.read_text())
        donor=json.loads(a.donor.read_text())
        teacher=LiveConnectionTeacher(donor['candidate'],staging,config,connection_only=True)
        teacher.env.reset(seed=0);teacher.env.settle();teacher.configure_task_open()
        teacher.report['teacher_input']={}
        sequence=teacher.staging_plan(None,None,require_dynamic=False)
        preflight=full_state_preflight(teacher,sequence)
        json.dump(preflight,stream,indent=2,allow_nan=False);stream.write('\n')
    print('COPIED',preflight['scope'],preflight['passed'],preflight['failure'],
          'minimum general clearance',preflight['minimum_measured_clearance_m'])
    return 0 if preflight['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
