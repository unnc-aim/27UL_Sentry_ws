"""Diagnostic geometry only. Unknown space and unmeasured body dimensions never authorize motion."""
import itertools
import math
import xml.etree.ElementTree as ET
import numpy as np


def transform(origin):
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
    r,p,y = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    out = np.eye(4)
    out[:3,:3] = [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                  [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    out[:3,3] = xyz
    return out


def model_bounds():
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    from xmacro.xmacro4sdf import XMLMacro4sdf
    from sdformat_tools.urdf_generator import UrdfGenerator
    macro=XMLMacro4sdf()
    macro.set_xml_file(str(Path(get_package_share_directory('pb2025_robot_description'))/'resource/xmacro/pb2025_sentry_robot.sdf.xmacro'))
    macro.generate(); generator=UrdfGenerator();generator.parse_from_sdf_string(macro.to_string())
    root=ET.fromstring(generator.to_string())
    poses={'base_footprint':np.eye(4)}
    pending=list(root.findall('joint'))
    while pending:
        rest=[]
        for joint in pending:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent in poses:poses[child]=poses[parent]@transform(joint.find('origin'))
            else:rest.append(joint)
        if len(rest)==len(pending):raise RuntimeError('Disconnected model geometry')
        pending=rest
    points=[];unsupported=[]
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry=collision.find('geometry');box=geometry.find('box');cylinder=geometry.find('cylinder');sphere=geometry.find('sphere')
            if box is not None:half=[float(v)/2 for v in box.get('size').split()]
            elif cylinder is not None:half=[float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2]
            elif sphere is not None:half=[float(sphere.get('radius'))]*3
            else:unsupported.append(link.get('name'));continue
            matrix=poses[link.get('name')]@transform(collision.find('origin'))
            for signs in itertools.product((-1,1),repeat=3):
                v=matrix@np.array([half[i]*signs[i] for i in range(3)]+[1.])
                points.append(v)
    if not points or unsupported:raise RuntimeError('Missing/unsupported collision geometry: '+str(unsupported))
    return [min(p[0] for p in points),max(p[0] for p in points),min(p[1] for p in points),max(p[1] for p in points)]


def first_contact(points, bounds, direction, margin=.10):
    """First pure-translation contact with observed points, not a free-space proof."""
    xmin,xmax,ymin,ymax=bounds
    intervals=[(xmin-margin,xmax+margin),(ymin-margin,ymax+margin)]
    earliest=None
    for point in points:
        enter,leave=0.,math.inf
        for value,speed,(lo,hi) in zip(point,direction,intervals):
            if speed==0:
                if value<lo or value>hi:enter=math.inf;break
            else:
                a,b=(value-hi)/speed,(value-lo)/speed
                enter=max(enter,min(a,b));leave=min(leave,max(a,b))
        if enter<=leave and math.isfinite(enter):earliest=enter if earliest is None else min(earliest,enter)
    return earliest


def circle_contact(points, direction, radius=.20, margin=.10):
    """Pure translation of the user-confirmed circular footprint against observed points."""
    length=math.hypot(*direction)
    if length==0:raise ValueError('Nonzero direction required')
    dx,dy=direction[0]/length,direction[1]/length
    expanded=radius+margin
    earliest=None
    for x,y in points:
        squared=x*x+y*y
        if squared<=expanded*expanded:return 0.
        projection=x*dx+y*dy
        perpendicular=max(0.,squared-projection*projection)
        if perpendicular>expanded*expanded:continue
        half=math.sqrt(max(0.,expanded*expanded-perpendicular))
        if projection+half<0:continue
        hit=max(0.,projection-half)
        earliest=hit if earliest is None else min(earliest,hit)
    return earliest


def assess(points):
    return {'radius_m':.20, 'radius_source':'user-confirmed physical radius; matches production navigation',
            'motion_authorized':False,'diagnostic_margin_m':.10,
            'limits':'Null contact means no observed blocker, NOT clear space. Braking distance requires physical validation.',
            'first_observed_contact_m':{name:circle_contact(points,direction) for name,direction in
              [('forward',(1,0)),('backward',(-1,0)),('left',(0,1)),('right',(0,-1))]}}
