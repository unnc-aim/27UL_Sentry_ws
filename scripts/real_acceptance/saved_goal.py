"""Validate selected goal against the frozen map and the active map-server grid."""
import hashlib
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


def validate_selected(selected, grid):
    if selected.get('frame') != 'map' or len(selected.get('targets', [])) != 1:
        raise ValueError('Select exactly one map goal')
    target = selected['targets'][0]
    if not all(math.isfinite(float(target[k])) for k in ('x', 'y')):
        raise ValueError('Non-finite goal')
    path = Path(selected['map_yaml'])
    meta = yaml.safe_load(path.read_text())
    image = path.parent/meta['image']
    if hashlib.sha256(image.read_bytes()).hexdigest() != selected['map_sha256']:
        raise ValueError('Map changed after goal selection')
    if meta.get('mode', 'trinary') != 'trinary' or abs(meta['origin'][2]) > 1e-6:
        raise ValueError('Expected unrotated trinary map')
    pixels = np.asarray(Image.open(image).convert('L'))
    if not meta.get('negate', 0) and np.any(pixels == 205) and meta['free_thresh'] > 50/255.:
        raise ValueError('Unknown grey cells would reload as free; rebuild/save with free_thresh=0.196')
    occupancy = pixels/255. if meta.get('negate', 0) else (255.-pixels)/255.
    expected = np.full(pixels.shape, -1, dtype=int)
    expected[occupancy < meta['free_thresh']] = 0
    expected[occupancy > meta['occupied_thresh']] = 100
    info = grid.info
    q = info.origin.orientation
    if (grid.header.frame_id != 'map' or pixels.shape != (info.height, info.width)
            or abs(info.resolution-meta['resolution']) > 1e-6
            or math.dist((info.origin.position.x, info.origin.position.y), meta['origin'][:2]) > 1e-5
            or abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(abs(q.w)-1.) > 1e-6
            or not np.array_equal(np.flipud(expected).ravel(), np.asarray(grid.data))):
        raise ValueError('Active map does not match selected goal map')
    return target
