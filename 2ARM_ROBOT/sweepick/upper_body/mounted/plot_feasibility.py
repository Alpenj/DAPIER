"""Top-view figure of the mounted pick feasibility (work6 grid). Robot geometry from the A 'mobile' model (MODEL_FOOTPRINT.json)."""
import json, sys
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

g = json.loads(Path(sys.argv[2] if len(sys.argv) > 2 else 'PICK_GRID_W6_v2.json').read_text()); fp = json.loads(Path('MODEL_FOOTPRINT.json').read_text())
yaw = int(sys.argv[1]) if len(sys.argv) > 1 else 0
step = 0.05
cells = {}
for c in g['cells']:
    if c['yaw_deg'] != yaw:
        continue
    k = (c['support'], c['shelf'])
    cells.setdefault(k, {}).setdefault((c['x'], c['y']), {})[c['side']] = c['ok']
P = lambda x, y: (-y, x)                                 # plot: horizontal = robot left to the left, vertical = forward up
panels = [(('floor', None), 'Floor'), (('shelf', 'LEFT_SIDE'), 'Shelf, robot LEFT side (top 0.104 m)'),
          (('shelf', 'RIGHT_SIDE'), 'Shelf, robot RIGHT side (top 0.104 m)'), (('shelf', 'FRONT'), 'Shelf, robot FRONT (top 0.104 m)')]
fig, axes = plt.subplots(1, 4, figsize=(22, 7.2))
colors = dict(both='#2e9e4f', left='#2f6fd6', right='#e07b1a')


def robot(ax):
    def box(name, **kw):
        b = fp[name]; (x0, y0), (x1, y1) = b['xy_min'], b['xy_max']
        px, py = P(x0, y1); ax.add_patch(Rectangle((px, py), (y1 - y0), (x1 - x0), **kw))
    box('tb3_base_link_collision', fc='#d9d9d9', ec='k', lw=1.2, zorder=3)
    for n in ('profile_front', 'profile_back'):
        box(n, fc='#9a9a9a', ec='k', lw=0.8, zorder=4)
    box('tb3_camera_link_collision', fc='#555', ec='k', lw=0.5, zorder=4)
    for n in ('tb3_wheel_left_link_collision', 'tb3_wheel_right_link_collision'):
        c = fp[n]; px, py = P(c['centre'][0], c['centre'][1]); ax.add_patch(Rectangle((px - 0.012, py - c['r']), 0.024, 2 * c['r'], fc='#333', zorder=4))
    c = fp['tb3_base_scan_collision']; ax.add_patch(Circle(P(*c['centre']), c['r'], fc='#bbb', ec='k', lw=0.6, zorder=4))
    for s, mk in (('left', '#2f6fd6'), ('right', '#e07b1a')):
        x, y, _ = fp[s + '_arm_base']; ax.plot(*P(x, y), marker='o', ms=11, mfc=mk, mec='k', zorder=6)
        ax.annotate(f'{s} arm', P(x, y), textcoords='offset points', xytext=(0, -16), ha='center', fontsize=8, zorder=6)
    ax.annotate('', xy=P(0.12, 0), xytext=P(-0.02, 0), arrowprops=dict(arrowstyle='->', lw=1.5), zorder=7)
    ax.text(*P(0.13, 0), 'forward', ha='center', va='bottom', fontsize=8, zorder=7)


for ax, (key, title) in zip(axes, panels):
    data = cells.get(key, {})
    if key[0] == 'shelf':
        sh = g['shelves'][key[1]]; (cx, cy), (hx, hy) = sh['centre_xy'], sh['half_size_xy']
        px, py = P(cx - hx, cy + hy); ax.add_patch(Rectangle((px, py), 2 * hy, 2 * hx, fc='#f3e3c9', ec='#8b6b3e', lw=1.2, zorder=1))
    for (x, y), sides in data.items():
        l, r = sides.get('left', False), sides.get('right', False)
        col = colors['both'] if l and r else colors['left'] if l else colors['right'] if r else None
        px, py = P(x, y)
        if col:
            ax.add_patch(Rectangle((px - step / 2, py - step / 2), step, step, fc=col, alpha=0.85, ec='white', lw=0.5, zorder=2))
        else:
            ax.plot(px, py, marker='x', ms=3, color='#c9c9c9', zorder=2)
    robot(ax)
    n = {s: sum(1 for v in data.values() if v.get(s)) for s in ('left', 'right')}
    ax.set_title(f'{title}\nleft arm {n["left"]} cells, right arm {n["right"]} cells', fontsize=10)
    ax.set_xlim(-0.52, 0.52); ax.set_ylim(-0.27, 0.50); ax.set_aspect('equal'); ax.grid(alpha=0.25)
    ax.set_xticks([-0.4, -0.2, 0, 0.2, 0.4]); ax.set_xticklabels(['0.4 L', '0.2 L', '0', '0.2 R', '0.4 R'])
    ax.set_xlabel('robot y [m] (left ← → right)'); ax.set_ylabel('robot x [m] (forward ↑)')
handles = [Rectangle((0, 0), 1, 1, fc=colors[k]) for k in ('left', 'right', 'both')] + [plt.Line2D([], [], marker='x', color='#c9c9c9', ls='')]
fig.legend(handles, ['left arm only', 'right arm only', 'both arms', 'planner refused'], loc='lower center', ncol=4, fontsize=10)
fig.suptitle(f'Mounted pick feasibility — block centre on a 5 cm grid, block yaw {yaw}°  |  existing SIM teacher planner (IK + path collision), OFFLINE MODEL, '
             'dimensions from the model (USER_UNVERIFIED)  |  shelves 2 cm outside the mount rails (y ±0.20 m) / front camera (x 0.085 m)', fontsize=10)
plt.tight_layout(rect=(0, 0.06, 1, 0.94))
out = Path(f'MOUNTED_PICK_FEASIBILITY_yaw{yaw}.png'); fig.savefig(out, dpi=110); print(out)
