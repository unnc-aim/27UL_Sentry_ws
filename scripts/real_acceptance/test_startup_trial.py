"""Read-only geometry check; no ROS or hardware initialized."""
from types import SimpleNamespace as Obj
from startup_trial import path_is_free
q=Obj(x=0.,y=0.,z=0.,w=1.)
grid=Obj(header=Obj(frame_id='map'),info=Obj(width=40,height=40,resolution=.05,
    origin=Obj(position=Obj(x=-1.,y=-1.),orientation=q)),data=[0]*1600)
assert path_is_free(grid,(0.,0.),(.2,0.))
grid.data[20*40+24]=100
assert not path_is_free(grid,(0.,0.),(.2,0.))
grid.data[20*40+24]=-1
assert not path_is_free(grid,(0.,0.),(.2,0.))
grid.data[20*40+24]=0
assert not path_is_free(grid,(0.,0.),(1.1,0.))
q.z=q.w=2**-.5
assert path_is_free(grid,(-1.5,0.),(-1.5,.2))
print('PASS: swept footprint, occupied, unknown, map boundary and rotated map origin')
