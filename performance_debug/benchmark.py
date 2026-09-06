"""Replay saved depth/geometry and benchmark pure functions; never initialize ROS."""
import importlib.util
import json
from pathlib import Path
import statistics
import subprocess
import time
from types import SimpleNamespace

import numpy as np
from cleanup_perception.grasp_candidates import body_candidates, choose_candidate
from cleanup_perception.obstacle_depth_node import depth_points
from robot_motion.policy import spin_velocity

root = Path(__file__).resolve().parents[1]
recording = root / 'cleanup_debug/candidate_hover_20260906_173957'
r = json.loads((recording / 'result.json').read_text())
d = np.load(recording / 'capture.npz')
args = (d['mask'], d['depth'], d['camera_k'], np.array(r['geometry']['rotation']),
        np.array(r['geometry']['translation']), .03)
info = SimpleNamespace(k=d['camera_k'], width=int(d['camera_size'][0]),
                       height=int(d['camera_size'][1]))


def bench(function, n):
    function()
    wall, cpu = [], []
    for _ in range(n):
        start, before = time.perf_counter(), time.process_time()
        function()
        cpu.append(1000 * (time.process_time() - before))
        wall.append(1000 * (time.perf_counter() - start))
    return {'n': n, 'wall_median_ms': statistics.median(wall),
            'wall_p95_ms': float(np.percentile(wall, 95)),
            'cpu_median_ms': statistics.median(cpu)}


results = {'depth_python_reference': bench(lambda: depth_points(d['depth'], info), 50),
           'body_candidates': bench(lambda: body_candidates(*args), 30)}
candidates = body_candidates(*args)
planner = root / 'install/pick_and_place/lib/pick_and_place/candidate_grasp_plan'
select = lambda: choose_candidate(candidates, r['geometry']['floor_link1'],
                                  r['start_joints'], planner)
results['candidate_cpp_subprocess'] = bench(select, 30)
selected, _ = select()
assert selected['pixel'] == [839., 543.]
assert abs(selected['plan']['pitch_degrees'] + 17.5) < 1e-8
results['successful_grasp_replay'] = {'pixel': selected['pixel'],
                                    'target_link1': selected['target_link1'],
                                    'pitch_degrees': selected['plan']['pitch_degrees']}
depth_file = root / 'performance_debug/depth.bin'
cloud_file = root / 'performance_debug/cloud.bin'
d['depth'].astype('<u2').tofile(depth_file)
h, w = d['depth'].shape
k = d['camera_k'].reshape(3, 3).copy()
k[0] *= w / info.width
k[1] *= h / info.height
results['depth_cpp'] = json.loads(subprocess.check_output(
    [str(root / 'performance_debug/depth_benchmark'), str(depth_file), str(w), str(h),
     *map(str, [k[0, 0], k[1, 1], k[0, 2], k[1, 2]]), str(cloud_file)], text=True))
np.testing.assert_allclose(np.fromfile(cloud_file, dtype='<f4').reshape(-1, 3),
                           depth_points(d['depth'], info), rtol=1e-6, atol=1e-6)
results['depth_cpp']['matches_python'] = True
# Paired old implementation rerun when the pre-edit snapshot is still available.
if Path('/tmp/grasp_candidates_baseline.py').exists():
    def module(name, path):
        spec = importlib.util.spec_from_file_location(name, path)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m
    old = module('old_candidates', '/tmp/grasp_candidates_baseline.py')
    old.jaw_center_sample = module('old_jaw', '/tmp/jaw_geometry_baseline.py').jaw_center_sample
    previous = old.body_candidates(*args)
    assert len(previous) == len(candidates)
    for before, after in zip(previous, candidates):
        for key in before:
            np.testing.assert_allclose(before[key], after[key], rtol=1e-10, atol=1e-12)
    results['body_candidates_before_paired'] = bench(lambda: old.body_candidates(*args), 10)
    results['body_candidates']['same_nine_candidates'] = True
(root / 'performance_debug/after.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
