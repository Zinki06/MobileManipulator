#!/usr/bin/env python3
# Copyright 2026 TurtleBot3 Project Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Record all discovered ROS topics through hardware shutdown."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time


def main():
    """Supervise rosbag without publishing motor commands or copying secrets."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    parser.add_argument('--launch-logs', required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--shutdown-tail', type=float, default=5.0)
    args = parser.parse_args()
    directory = Path(args.root).expanduser().resolve() / (
        datetime.now().strftime('full_%Y%m%d_%H%M%S_') + str(os.getpid()))
    directory.mkdir(parents=True)
    logs = Path(args.launch_logs).resolve()
    (directory / 'launch_logs').symlink_to(logs, target_is_directory=True)
    if Path(args.profile).is_file():
        shutil.copyfile(args.profile, directory / 'performance.yaml')
    command = ['ros2', 'bag', 'record', '--all', '--include-hidden-topics',
               '--max-bag-size', '1073741824', '--max-cache-size', '33554432',
               '--polling-interval', '100', '-o', str(directory / 'bag')]
    manifest = {
        'started_unix': time.time(), 'command': command,
        'launch_logs': str(logs), 'shutdown_tail_seconds': args.shutdown_tail,
        'limitations': [
            'Topics are recorded after DDS discovery; delivery is not guaranteed.',
            'ROS 2 Humble bag does not record service request/response traffic.',
            'Motor internal error, voltage, temperature and torque registers are '
            'not exposed by the current hardware driver.',
            'Action feedback/status are topics; goals/results require node logs.',
        ],
    }

    def save():
        (directory / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')

    stop_at = None

    def stop(_number, _frame):
        nonlocal stop_at
        if stop_at is None:
            stop_at = time.monotonic() + args.shutdown_tail
            print('[DEBUG_RECORDING] Capturing hardware shutdown tail', flush=True)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    save()
    print(f'[DEBUG_RECORDING] {directory}', flush=True)
    forced = False
    with (directory / 'recorder.log').open('w') as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            while child.poll() is None:
                if stop_at is not None and time.monotonic() >= stop_at:
                    break
                time.sleep(0.1)
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    forced = True
                    os.killpg(child.pid, signal.SIGTERM)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
            manifest.update(ended_unix=time.time(), recorder_returncode=child.returncode,
                            forced_shutdown=forced,
                            metadata_present=(directory / 'bag/metadata.yaml').is_file())
            save()
    if child.returncode or forced or stop_at is None or not manifest['metadata_present']:
        raise SystemExit(f'[DEBUG_RECORDING_FAILED] Inspect {directory}/recorder.log')
    print(f'[DEBUG_RECORDING_COMPLETE] {directory}', flush=True)


if __name__ == '__main__':
    main()
