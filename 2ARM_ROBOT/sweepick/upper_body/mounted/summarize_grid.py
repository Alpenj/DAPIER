"""Summarize the work6 pick feasibility grid: per support / shelf / arm / block yaw, which block centres the EXISTING teacher
planner accepted (pregrasp -> grasp -> lift, single segments, its own path collision check). Text maps: x forward (rows,
far at the top), y left (columns, left at the left); 'O' feasible, '.' refused, ' ' not on that support."""
import json, sys
from collections import Counter, defaultdict
from pathlib import Path

d = json.loads(Path(sys.argv[1]).read_text())
xs, ys = d['grid']['x'], d['grid']['y']
groups = defaultdict(dict)
for c in d['cells']:
    groups[(c['support'], c['shelf'], c['side'], c['yaw_deg'])][(c['x'], c['y'])] = c
summary = {}
for key in sorted(groups, key=lambda k: (k[0], str(k[1]), k[2], k[3])):
    cells = groups[key]; ok = [c for c in cells.values() if c['ok']]
    name = f"{key[0]}{'' if key[1] is None else ':' + key[1]} {key[2]} yaw{key[3]}"
    reasons = Counter((c.get('why') or ['?'])[0].split(':', 2)[-1].strip()[:60] for c in cells.values() if not c['ok'])
    summary[name] = dict(cells=len(cells), feasible=len(ok),
                         x_range=[min(c['x'] for c in ok), max(c['x'] for c in ok)] if ok else None,
                         y_range=[min(c['y'] for c in ok), max(c['y'] for c in ok)] if ok else None,
                         grasps=dict(Counter(f"tilt{c['tilt_deg']:.0f}/{c['jaw']}" for c in ok)), top_refusals=reasons.most_common(3))
    lines = []
    for x in sorted(xs, reverse=True):
        row = ''.join(' ' if (x, y) not in cells else ('O' if cells[(x, y)]['ok'] else '.') for y in sorted(ys, reverse=True))
        if row.strip():
            lines.append(f'{x:+.2f} {row}')
    summary[name]['map'] = lines
    print(f"== {name}: {len(ok)}/{len(cells)} feasible  x{summary[name]['x_range']} y{summary[name]['y_range']}")
    if len(sys.argv) > 2:
        print('\n'.join(lines))
Path(sys.argv[1]).with_name(Path(sys.argv[1]).stem + '_SUMMARY.json').write_text(json.dumps(dict(elapsed_s=d['elapsed_s'], shelves=d['shelves'], grid=d['grid'], groups=summary), indent=1) + '\n')
