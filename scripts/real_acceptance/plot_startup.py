#!/usr/bin/env python3
"""Render saved local occupancy map and estimated navigation trace."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('folder',type=Path)
args=parser.parse_args();folder=args.folder
name=next((name for name in ['map','scanned_map','partial_map'] if (folder/(name+'.yaml')).exists()),None)
if name is None:raise SystemExit('No saved occupancy map')
meta=yaml.safe_load((folder/(name+'.yaml')).read_text());img=mpimg.imread(folder/meta['image'])
h,w=img.shape;ox,oy,yaw=meta['origin'];r=meta['resolution']
if abs(yaw)>1e-6:raise SystemExit('Rotated map origin requires transformed plot axes')
fig,ax=plt.subplots(figsize=(8,7));ax.imshow(img,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+w*r,oy,oy+h*r])
summary=json.loads((folder/'summary.json').read_text());start=summary.get('initial_pose_map')
if start:ax.plot(*start[:2],'bo',label='Initial pose (estimated)')
if (folder/'navigation_trial.json').exists():
    nav=json.loads((folder/'navigation_trial.json').read_text());trace=nav['trace']
    if trace:ax.plot([v['pose'][0] for v in trace],[v['pose'][1] for v in trace],'b-',label='Estimated path')
    for goal in nav['waypoints']:ax.plot(*goal['target'],'r*',markersize=12,label='Map goal')
if start:
    ax.set_xlim(start[0]-2,start[0]+2);ax.set_ylim(start[1]-2,start[1]+2)
ax.set_aspect('equal');ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)');ax.set_title('Local map near robot (4 m view)');ax.legend();ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(folder/'map_preview.png',dpi=160);plt.close(fig)
print(folder/'map_preview.png')
