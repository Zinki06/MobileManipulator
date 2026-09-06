"""Validate process CPU accounting without sampling another process or its arguments."""

from robot_motion.process_metrics import ProcessMetrics


def test_cpu_deltas_rss_and_pid_reuse(tmp_path, monkeypatch):
    """CPU starts unknown, counts core time, and resets when a PID is recycled."""
    now = [10.]
    monkeypatch.setattr('robot_motion.process_metrics.time.monotonic', lambda: now[0])
    process = tmp_path / '42'
    process.mkdir()
    (process / 'environ').write_bytes(b'ROS_DISTRO=humble\x00')
    fields = ['0'] * 30
    fields[0], fields[11], fields[12], fields[19], fields[21] = 'S', '100', '20', '10', '256'

    def write():
        (process / 'stat').write_text('42 (fake node) ' + ' '.join(fields))

    write()
    metrics = ProcessMetrics(tmp_path)
    metrics.ticks, metrics.page_size = 100, 4096
    assert metrics.sample()['processes'][0]['cpu_percent_one_core'] is None
    now[0] += 5
    fields[11] = '200'
    write()
    result = metrics.sample()['processes'][0]
    assert abs(result['cpu_percent_one_core'] - 20.) < 1e-9
    assert result['rss_mib'] == 1.
    assert result['executable'] == 'fake node'
    fields[19] = '999'
    write()
    assert metrics.sample()['processes'][0]['cpu_percent_one_core'] is None
