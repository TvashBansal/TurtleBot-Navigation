"""Rerun an archived experiment from its saved parameters and compare key metrics."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def reproduce(archive: Path, output: Path) -> dict:
    archive = archive.resolve()
    output = output.resolve()
    manifest = json.loads((archive / 'manifest.json').read_text(encoding='utf-8'))
    backend, kind = manifest['backend'], manifest['kind']
    if kind == 'tests':
        command = [sys.executable, 'scripts/record_tests.py', '--out', str(output)]
    else:
        command = [sys.executable, '-m', 'turtlebot_integrated.cli', 'run' if kind == 'run' else 'suite',
                   '--backend', backend, '--config', str(archive / 'config_snapshot.json'), '--out', str(output)]
    if kind == 'run':
        command += ['--scenario', manifest['identity']['scenario'], '--seed', str(manifest['identity']['seed'])]
        if (archive / 'demo_capture.mp4').is_file():
            command.append('--video')
    elif kind == 'suite':
        command += ['--seeds', str(manifest['identity']['seeds'])]
        if json.loads((archive / 'suite_metadata.json').read_text(encoding='utf-8')).get('video_requested'):
            command.append('--video')
    subprocess.run(command, cwd=ROOT, check=True)
    filename = 'summary.json' if kind == 'run' else 'aggregate.json' if kind == 'suite' else 'test_metadata.json'
    previous = json.loads((archive / filename).read_text(encoding='utf-8'))
    current = json.loads((output / filename).read_text(encoding='utf-8'))
    if kind == 'tests':
        previous = {'exit_code': previous['exit_code'], **previous['counts']}
        current = {'exit_code': current['exit_code'], **current['counts']}
    keys = (['success', 'collision', 'duration_s', 'position_error_m', 'rmse_x_m', 'rmse_y_m', 'rmse_heading_rad']
            if kind == 'run' else
            ['count', 'success_rate', 'collision_rate', 'mean_position_error_m', 'mean_rmse_xy_m']
            if kind == 'suite' else ['exit_code', 'tests', 'failures', 'errors', 'skipped'])
    changes = {}
    for key in keys:
        before, after = previous[key], current[key]
        numeric = isinstance(before, (int, float)) and not isinstance(before, bool)
        changes[key] = {'archived': before, 'reproduced': after}
        if numeric:
            changes[key]['difference'] = round(float(after) - float(before), 8)
            changes[key]['same_within_1e-6'] = math.isclose(float(before), float(after), abs_tol=1e-6, rel_tol=1e-6)
        else:
            changes[key]['same'] = before == after
    comparison = {
        'archive': str(archive.relative_to(ROOT)) if archive.is_relative_to(ROOT) else str(archive),
        'backend': backend,
        'code_revision_at_original_run': manifest['git_commit_at_run'],
        'caveat': 'Running current checkout may differ from the recorded code revision; numerical differences do not automatically mean a failed experiment.',
        'metrics': changes,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / 'reproduction_comparison.json').write_text(json.dumps(comparison, indent=2) + '\n', encoding='utf-8')
    return comparison


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--out', type=Path, help='Directory for the newly generated results (under outputs/)')
    args = parser.parse_args()
    dest = args.out or (ROOT / 'outputs' / f'recheck_{args.archive.name}')
    print(json.dumps(reproduce(args.archive, dest), indent=2))


if __name__ == '__main__':
    main()
