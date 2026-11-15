#!/usr/bin/env python3
"""Merge the route preflight patch into a customized local NIIHAN workspace.

Uses git merge-file in print mode; computes ALL changes before writing any.
Preserves untouched local assets; aborts without writing if any conflict exists.
Requires origin refs fetched first. Run only with simulation stopped.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile


BASE = 'origin/fix/seaport-site-containment-20261115'
FEATURE = 'origin/fix/nav2-route-preflight-visualization-20261115'
FILES = (
    'src/niihan_dashboard/niihan_dashboard/ros_bridge.py',
    'src/niihan_dashboard/niihan_dashboard/dashboard_node.py',
    'src/niihan_dashboard/niihan_dashboard/route_policy.py',
    'src/niihan_dashboard/web/js/dashboard.js',
    'src/niihan_dashboard/web/js/viewer2d.js',
    'src/niihan_dashboard/test/test_navigation_results.py',
    'src/niihan_dashboard/test/test_route_policy.py',
    'src/niihan_slam/config/nav2_3d_jazzy.yaml',
    'src/niihan_slam/niihan_slam/grid.py',
    'src/niihan_slam/test/test_grid.py',
    'docs/NAV2_ROUTE_PREFLIGHT_AND_VISUALIZATION_20261115.md',
)


def git_show(ref, path):
    cmd = subprocess.run(['git', 'show', f'{ref}:{path}'],
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if cmd.returncode:
        return None
    return cmd.stdout


def main():
    root = Path.cwd()
    if not (root / 'src/niihan_slam').is_dir() or not (root / 'src/niihan_dashboard').is_dir():
        sys.exit('ERROR: Run from ~/surveillance_project/niihan_ws')
    if subprocess.run(['git', 'rev-parse', '--is-inside-work-tree'],
                      stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode:
        sys.exit('ERROR: Not a Git repository.')
    plans = {}
    with tempfile.TemporaryDirectory(prefix='niihan_nav_merge_') as tmp:
        tmp = Path(tmp)
        for index, name in enumerate(FILES):
            local_path = root / name
            before = git_show(BASE, name)
            after = git_show(FEATURE, name)
            if after is None:
                sys.exit(f'ERROR: Missing feature ref/file {FEATURE}:{name}; fetch both branches.')
            if before is None:
                if local_path.exists():
                    if local_path.read_bytes() != after:
                        sys.exit(f'CONFLICT: new patch file already exists locally: {name}')
                    merged = after
                else:
                    merged = after
            else:
                if not local_path.is_file():
                    sys.exit(f'CONFLICT: existing base file is missing locally: {name}')
                base_temp = tmp / f'{index}_base'
                new_temp = tmp / f'{index}_new'
                base_temp.write_bytes(before)
                new_temp.write_bytes(after)
                result = subprocess.run(
                    ['git', 'merge-file', '-p', str(local_path),
                     str(base_temp), str(new_temp)],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                if result.returncode != 0:
                    sys.exit(
                        f'CONFLICT: Could not automatically merge {name}. No files changed.\n'
                        f'{result.stderr.decode(errors="replace")}')
                merged = result.stdout
            if not local_path.is_file() or local_path.read_bytes() != merged:
                plans[name] = merged

        # Static guard against accidentally loading an unsafe old profile.
        yaml_text = plans.get(
            'src/niihan_slam/config/nav2_3d_jazzy.yaml',
            (root / 'src/niihan_slam/config/nav2_3d_jazzy.yaml').read_bytes(),
        ).decode()
        if 'allow_unknown: false' not in yaml_text or 'max_planning_time: 2.0' not in yaml_text:
            sys.exit('ERROR: Unsafe/missing Nav2 safety profile after merge. No files changed.')

        if not plans:
            print('Nav2 preflight patch already installed; no changes necessary.')
            return

        backup = (Path.home() / 'surveillance_project' / 'backups' /
                  ('nav2_preflight_' + datetime.now().strftime('%Y%m%d_%H%M%S')))
        for name in plans:
            src = root / name
            if src.is_file():
                dest = backup / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dest)
        # Compute everything and back up originals BEFORE any writes.
        for name, data in plans.items():
            dest = root / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            staged = dest.with_name(dest.name + '.nav2_staged')
            staged.write_bytes(data)
            os.replace(staged, dest)

        print(f'Applied Nav2 route preflight patch to {len(plans)} files.')
        print(f'Original copies saved at: {backup}')
        print('Your SDF, GLB, start script and unrelated assets were not changed.')
        print('Next: build niihan_dashboard and niihan_slam, then run tests before driving.')


if __name__ == '__main__':
    main()
