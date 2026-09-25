"""Reject stale map selections and different live maps before actuator commands."""
import copy
import hashlib
import tempfile
from pathlib import Path
from types import SimpleNamespace as Obj
from PIL import Image
from saved_goal import validate_selected

with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp)/'map.yaml';image=path.with_suffix('.pgm')
    pixels=Image.new('L',(3,2));pixels.putdata([0,254,205,254,0,254]);pixels.save(image)
    path.write_text('image: map.pgm\nresolution: 0.05\norigin: [-1, -2, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    selected={'frame':'map','targets':[{'x':0.,'y':0.}], 'map_yaml':str(path),
              'map_sha256':hashlib.sha256(image.read_bytes()).hexdigest()}
    grid=Obj(header=Obj(frame_id='map'),info=Obj(width=3,height=2,resolution=.05,
        origin=Obj(position=Obj(x=-1.,y=-2.),orientation=Obj(x=0.,y=0.,z=0.,w=1.))),
        data=[0,100,0,100,0,-1])
    validate_selected(selected,grid)
    for bad in ['occupancy','origin','dimensions','frame','hash','nan']:
        s=copy.deepcopy(selected);g=copy.deepcopy(grid)
        if bad=='occupancy':g.data[0]=100
        if bad=='origin':g.info.origin.position.x+=.1
        if bad=='dimensions':g.info.width=2
        if bad=='frame':g.header.frame_id='odom'
        if bad=='hash':s['map_sha256']='wrong'
        if bad=='nan':s['targets'][0]['x']=float('nan')
        try:validate_selected(s,g)
        except ValueError:pass
        else:raise AssertionError('Unsafe selection accepted: '+bad)
    path.write_text(path.read_text().replace('free_thresh: 0.196','free_thresh: 0.25'))
    grid.data[-1]=0  # A loader would wrongly label grey 205 as free with this threshold.
    try:validate_selected(selected,grid)
    except ValueError:pass
    else:raise AssertionError('Unknown-to-free map conversion accepted')
print('PASS: raster orientation, occupancy, origin, dimensions, frame, map hash and finite target')
