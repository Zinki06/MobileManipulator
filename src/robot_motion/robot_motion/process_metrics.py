"""Sample ROS process CPU time without shell commands or reading command arguments."""

import os
from pathlib import Path
import time


class ProcessMetrics:
    """Report per-process core utilization and RSS from Linux procfs deltas."""

    def __init__(self, root='/proc'):
        self.root = Path(root)
        self.previous = {}
        self.last = time.monotonic()
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.page_size = os.sysconf('SC_PAGE_SIZE')

    def sample(self):
        """Return ROS executable names, CPU percentages and resident memory only."""
        now = time.monotonic()
        duration = max(now - self.last, 1e-6)
        current, records = {}, []
        for directory in self.root.iterdir():
            if not directory.name.isdecimal():
                continue
            try:
                # Environment values select ROS processes; values are never recorded.
                environment = (directory / 'environ').read_bytes()
                if b'ROS_DISTRO=' not in environment:
                    continue
                stat = (directory / 'stat').read_text()
                end = stat.rfind(')')
                name = stat[stat.index('(') + 1:end]
                fields = stat[end + 2:].split()
                key = (int(directory.name), fields[19])  # PID plus start time.
                cpu = (int(fields[11]) + int(fields[12])) / self.ticks
                current[key] = cpu
                previous = self.previous.get(key)
                records.append({'pid': key[0], 'executable': name,
                                'cpu_percent_one_core': None if previous is None else
                                max(0., 100 * (cpu - previous) / duration),
                                'rss_mib': int(fields[21]) * self.page_size / 1048576})
            except (OSError, ValueError, IndexError):
                continue
        self.previous, self.last = current, now
        return {'sample_seconds': duration, 'processes': records}
