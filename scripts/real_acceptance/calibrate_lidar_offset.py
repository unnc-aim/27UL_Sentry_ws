#!/usr/bin/env python3
"""Estimate planar lidar lever-arm correction from a stationary-body yaw sweep.

This is a candidate calibration; validate on an independent full sweep before use
in competition. Does not modify robot configuration or publish ROS commands.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def fit(rows):
    rows=[r for r in rows if r.get('gimbal_yaw') is not None]
    angles=np.unwrap([r['gimbal_yaw'] for r in rows])
    if len(rows)<30 or np.ptp(angles)<1.:raise ValueError('Insufficient yaw sweep')
    if any(abs(v)>.001 for r in rows for v in r['chassis']):
        raise ValueError('Nonzero chassis command; stationary-body fit invalid')
    a=[];b=[]
    for row in rows:
        c=math.cos(row['gimbal_yaw']);s=math.sin(row['gimbal_yaw'])
        a.extend([[1,0,c,-s],[0,1,s,c]]);b.extend(row['position'])
    a=np.array(a);b=np.array(b)
    x,_,_,_=np.linalg.lstsq(a,b,rcond=None)
    residual=(a@x-b).reshape(-1,2)
    rms=float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))
    if rms>.015 or np.linalg.cond(a)>20 or np.linalg.norm(x[2:])>.15:
        raise ValueError('Unreliable lever-arm fit')
    return {'correction_xy_m':x[2:].tolist(),'fit_rms_m':rms,
            'sweep_rad':float(np.ptp(angles)),'independent_validation_passed':False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--stationary-confirmed',action='store_true')
    args=parser.parse_args()
    result=fit(json.loads(args.trace.read_text())['trace'])
    result.update(source=str(args.trace.resolve()),stationary_confirmed=args.stationary_confirmed)
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
