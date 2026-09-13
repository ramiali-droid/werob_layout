#!/usr/bin/env python3
"""Plot the measured acceptance trajectories over the SDF collision geometry."""
import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT = Path(__file__).resolve().parents[1]


def corners(x, y, yaw, hx, hy):
    return [(x+a*math.cos(yaw)-b*math.sin(yaw), y+a*math.sin(yaw)+b*math.cos(yaw))
            for a,b in ((-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy))]


def trajectory(mode):
    with (ROOT/'validation'/f'{mode}_trajectory.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    return [tuple(float(row[key]) for key in ('x_m','y_m','yaw_rad')) for row in rows]


def main():
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={'width_ratios': [2.2, 1]})
    for ax in axes:
        ax.set_aspect('equal')
        ax.set_facecolor('#f3f5ee')
        ax.grid(alpha=0.18)
        ax.set_xlabel('World x (m)')
        ax.set_ylabel('World y (m)')
        for model in ET.parse(ROOT/'src/patrol_simulation/worlds/pilot_world.sdf').findall('.//world/model'):
            for link in model.findall('link'):
                size = link.findtext('collision/geometry/box/size')
                if not size:
                    continue
                x,y,z,_,_,yaw = map(float,link.findtext('pose','0 0 0 0 0 0').split())
                sx,sy,sz = map(float,size.split())
                if z-sz/2 >= 0.70:
                    continue
                color = '#e3ac26' if 'door_jamb' in link.attrib['name'] else '#77818a'
                if 'gate' in link.attrib['name']:
                    color = '#b8554e'
                ax.add_patch(Polygon(corners(x,y,yaw,sx/2,sy/2), color=color, alpha=0.85))
        data = trajectory('husky_mixed')
        ax.plot([r[0] for r in data],[r[1] for r in data], color='#2365aa', linewidth=2,
                label='Husky patrol: 2 completed rounds')
        ax.add_patch(Polygon(corners(-3.5,-5.7,0,.4,.1), color='#8d542b', label='Side pallet'))
        ax.add_patch(Polygon(corners(8,-.85,0,.4,.1), color='#8d542b', label='Outdoor pallet'))
    blocked = trajectory('husky_blocking')[-1]
    axes[1].add_patch(Polygon(corners(-3.5,-5,0,.4,.1), facecolor='#d87973', edgecolor='#a23f37',
                             alpha=.65, label='Pallet in Husky blocking test'))
    axes[1].add_patch(Polygon(corners(*blocked,.56,.35), fill=False, edgecolor='#a23f37',
                             linewidth=2, label='Husky stopped before contact'))
    axes[1].plot(blocked[0],blocked[1], 'x', color='#a23f37', markersize=8)
    axes[0].annotate('Dock', (0,0), xytext=(-1,1))
    axes[0].annotate('Indoor checkpoint', (-5.5,-5), xytext=(-8,-4))
    axes[0].annotate('Bottleneck: 1.2 m', (.9,-5), xytext=(2,-7.8))
    axes[0].set_xlim(-10,15)
    axes[0].set_ylim(-8.5,4)
    axes[0].set_title('Accepted route, measured under wheel physics')
    axes[0].legend(loc='upper left', fontsize=8)
    axes[1].set_xlim(-4.4,-1.5)
    axes[1].set_ylim(-6.3,-3.8)
    axes[1].set_title('Separate side-pallet and blocked-path tests')
    axes[1].legend(loc='upper center', fontsize=7)
    fig.tight_layout()
    fig.savefig(ROOT/'validation/route_verification.png', dpi=180)


if __name__ == '__main__':
    main()
