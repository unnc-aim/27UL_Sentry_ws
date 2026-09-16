"""Plot independent Gazebo trajectories and target errors from completed runs."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from scenario import BOXES, GOALS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    fig, (map_ax, error_ax) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [1.4, 1]})
    for x, y, sx, sy in BOXES:
        map_ax.add_patch(Rectangle((x-sx/2, y-sy/2), sx, sy, color='#465366'))
    for name, (x, y, _) in GOALS.items():
        map_ax.plot(x, y, 'kx', markersize=8)
        map_ax.annotate(name, (x, y), xytext=(5, 7), textcoords='offset points')
    for run in args.runs:
        result = json.loads((run/'result.json').read_text())
        if not result.get('passed'):
            raise ValueError(f'{run} is not a passing run')
        config = json.loads((run/'run_config.json').read_text())
        rows = list(csv.DictReader((run/'result.csv').open()))
        label = f"start yaw {config['spawn_yaw']:.1f}, spin {config['nav_spin_speed']:.1f} rad/s"
        line, = map_ax.plot([float(r['world_x']) for r in rows], [float(r['world_y']) for r in rows], label=label)
        error_ax.plot([r['name'] for r in result['goals']], [r['error_m'] for r in result['goals']],
                      'o-', label=label, color=line.get_color())
    map_ax.set(xlim=(-5.3, 5.3), ylim=(-4.3, 4.3), xlabel='Map/world X (m)', ylabel='Map/world Y (m)',
               title='Gazebo ground-truth trajectories')
    map_ax.set_aspect('equal')
    map_ax.legend(fontsize=8, loc='lower right')
    error_ax.axhline(.35, color='#bb3333', linestyle='--', label='Acceptance limit: 0.35 m')
    error_ax.set(ylabel='Ground-truth arrival error (m)', ylim=(0, .4), title='Same map landmarks, different orientations')
    error_ax.grid(alpha=.2)
    error_ax.legend(fontsize=8)
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)


if __name__ == '__main__':
    main()
