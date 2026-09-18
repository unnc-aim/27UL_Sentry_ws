"""Plot a saved static scan and model envelope; no ROS or control interfaces."""
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle

folder=Path(__file__).parent/'results'
scene=json.loads((folder/'small_scene.json').read_text())
plan=json.loads((folder/'small_planning.json').read_text())
fig,ax=plt.subplots(figsize=(7,6))
points=scene.get('endpoints',[])
if points:ax.scatter([p[0] for p in points],[p[1] for p in points],s=5,label='Observed scan returns')
x0,x1,y0,y1=scene['directional_geometry']['model_bounds_xy_m']
ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,color='black',label='Model envelope (unverified)'))
ax.add_patch(Rectangle((x0-.1,y0-.1),x1-x0+.2,y1-y0+.2,fill=False,ls='--',color='orange',label='Diagnostic margin: 0.10 m'))
ax.add_patch(Circle((0,0),.2,fill=False,color='red',label='Existing Nav2 radius: 0.20 m'))
if 'base_pose_map' in plan:
 x,y,a=plan['base_pose_map']
 for path in plan['paths']:
  pts=path.get('points',[])
  if pts:
   local=[((px-x)*math.cos(a)+(py-y)*math.sin(a),-(px-x)*math.sin(a)+(py-y)*math.cos(a)) for px,py in pts]
   ax.plot([p[0] for p in local],[p[1] for p in local],label=path['direction']+' plan only')
ax.set(xlabel='base_footprint X (m)',ylabel='base_footprint Y (m)',title='Static diagnostics: missing returns are unknown',xlim=(-1.5,2),ylim=(-1.5,1.5))
ax.set_aspect('equal');ax.grid(alpha=.3);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(folder/'small_scene.png',dpi=160)
