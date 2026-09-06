"""Replay recorded height alignment and complete grasp plans without ROS or hardware."""
import copy
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from cleanup_perception.floor_alignment import align_candidate_heights

root = Path('/home/user/turtlebot3_ws')
run = root / 'cleanup_debug/run_20260906_192010_47639'
planner = root / 'install/pick_and_place/lib/pick_and_place/candidate_grasp_plan'
results = []
for path in sorted(run.glob('*observation/*.npz')):
    data = np.load(path)
    original = json.loads(path.with_suffix('.candidates.json').read_text())
    candidates = copy.deepcopy(original['evaluated'])
    started = time.perf_counter()
    fit = align_candidate_heights(candidates, data['mask'], data['depth'],
        data['camera_k'].reshape(3, 3), np.array(original['rotation']),
        np.array(original['translation']), original['floor_link1'])
    elapsed = time.perf_counter() - started
    assert fit['applied'], fit
    selected = next(c for c in candidates if c['pixel'] == original['selected']['pixel'])
    before = original['selected']['observed_link1'][2] - original['floor_link1']
    after = selected['observed_link1'][2] - original['floor_link1']
    approaches = []
    for standoff in np.arange(.30, .179, -.02):
        rows = [[standoff + .092 + .03 + error, 0., height + original['floor_link1'],
                 original['floor_link1']] + original['start_joints']
                for height in (before, after) for error in (0., .02)]
        request = ''.join(' '.join(map(str, row)) + '\n' for row in rows)
        response = subprocess.run([str(planner)], input=request, text=True,
                                  check=True, capture_output=True)
        plans = [json.loads(line) for line in response.stdout.splitlines()]
        assert not plans[0]['feasible'] and not plans[1]['feasible']
        approaches.append({'base_standoff_m': float(standoff),
            'before': [p['feasible'] for p in plans[:2]],
            'after': [p['feasible'] for p in plans[2:]],
            'after_pitch_degrees': [p.get('pitch_degrees') for p in plans[2:]]})
    assert any(all(a['after']) for a in approaches)
    results.append({'capture': str(path.relative_to(root)), 'fit': fit,
        'alignment_seconds': elapsed, 'height_before_m': before,
        'height_after_m': after, 'selected_pixel': selected['pixel'], 'approaches': approaches,
        'assumption': 'Planar base; measured arm mount base->link1=(-0.092,0,0.101); '
                      'fixed +0.03m forward correction; existing 0.02m arrival error check'})
output = root / 'performance_debug/grasp_floor_replay.json'
output.write_text(json.dumps(results, indent=2) + '\n')
for row in results:
    print(row['capture'], 'height mm', row['height_before_m']*1000,
          '->', row['height_after_m']*1000, 'alignment ms', row['alignment_seconds']*1000)
print(output)
