#!/usr/bin/env python3
"""One-session acceptance sequence. Requires an already running isolated navigation stack.

Real commands only through the original Hub and RC authority. Not a certification
of physical emergency-stop braking, saved-map global localization or moving-obstacle avoidance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import signal
import sys
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args=parser.parse_args()
    if not args.execute:parser.error('--execute required after on-site readiness')
    folder=Path(__file__).resolve().parent
    out=folder/'results'/time.strftime('full_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,SENTRY_ACCEPTANCE_OUTPUT=str(out))
    report={'status':'running','physical_acceptance_passed':False,'radius_m':.2,'stages':[],
            'pending':['Physical emergency-stop braking with operator',
                       'Saved-map global localization and symmetry resolution',
                       'Physical obstruction/avoidance and data-loss stop test']}
    summary=out/'summary.json'
    def save():summary.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    def stage(name,script,arguments,filename,timeout):
        report['active_stage']=name;save()
        path=out/filename
        path.unlink(missing_ok=True)
        # Child scripts enforce RC freshness, true emergency state, speed and travel limits.
        child=subprocess.Popen([sys.executable,str(folder/script),'--execute',*arguments],env=env)
        try:
            code=child.wait(timeout=timeout)
        except (subprocess.TimeoutExpired,KeyboardInterrupt):
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill();child.wait()
            raise
        path=out/filename
        evidence=json.loads(path.read_text()) if path.exists() else {'error':'No completion report'}
        (out/(name+'.json')).write_text(json.dumps(evidence,indent=2)+'\n')
        report['stages'].append({'name':name,'exit_code':code,'evidence':name+'.json'})
        save()
        if code:raise RuntimeError(name+' failed; subsequent motion skipped')
        return evidence
    save()
    try:
        baseline=stage('baseline_rectangle','navigation_trial.py',['--course','rectangle'],'navigation_trial.json',150)
        landmarks=out/'landmarks.json'
        landmarks.write_text(json.dumps({'frame':'map','session':out.name,'targets':[
            {'name':w['name'],'x':w['target'][0],'y':w['target'][1]} for w in baseline['waypoints']]},indent=2)+'\n')
        yaw=stage('short_rotation','motion_probe.py',['--axis','yaw','--speed','.4','--duration','1.5'],'motion_probe.json',20)
        if abs(yaw.get('measured_yaw_change_rad',0))<.10:raise RuntimeError('Rotation response too small to validate')
        stage('changed_heading_rectangle','navigation_trial.py',['--landmarks',str(landmarks)],'navigation_trial.json',150)
        rotation=stage('startup_two_turns','motion_probe.py',['--axis','yaw','--speed','.4','--turns','2'],'motion_probe.json',135)
        if rotation.get('startup_turns_completed')!=2 or abs(rotation.get('measured_yaw_change_rad',0))<4*math.pi:
            raise RuntimeError('Two measured rotations not completed')
        stage('post_startup_rectangle','navigation_trial.py',['--landmarks',str(landmarks)],'navigation_trial.json',150)
        report['status']='local_sequence_passed'
    except (Exception,KeyboardInterrupt) as exc:
        report['status']='incomplete';report['reason']=str(exc) or 'Interrupted'
    finally:
        report.pop('active_stage',None);save();print('Acceptance report:',summary)
    return 0 if report['status']=='local_sequence_passed' else 1


if __name__=='__main__':raise SystemExit(main())
