"""Copy selected experiment output into immutable, Git-tracked results evidence.

Run from the repository root: python scripts/archive_run.py outputs/nominal --name nominal_seed7
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
SIZE_LIMIT_BYTES = 49 * 1024 * 1024


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as src:
        for chunk in iter(lambda: src.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def git_head() -> str | None:
    p = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True, check=False)
    return p.stdout.strip() if p.returncode == 0 else None


def archive(source: Path, name: str, allow_large: bool = False) -> Path:
    source = source.resolve()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('Archive name may contain only letters, digits, hyphen and underscore (max 80 chars).')
    if not source.is_dir():
        raise ValueError(f'Experiment output directory not found: {source}')
    if (source / 'summary.json').is_file():
        kind = 'run'
        info = json.loads((source / 'summary.json').read_text(encoding='utf-8'))
        required = ('config_snapshot.json', 'run_metadata.json', 'trajectory.json')
        identity = {'scenario': info['scenario'], 'seed': info['seed']}
    elif (source / 'aggregate.json').is_file():
        kind = 'suite'
        info = json.loads((source / 'aggregate.json').read_text(encoding='utf-8'))
        required = ('config_snapshot.json', 'suite_metadata.json')
        if not list(source.rglob('summary.json')):
            raise ValueError('Suite has no individual experiment summary files.')
        identity = {'count': info['count'], 'seeds': info['count'] // 5}
        if identity['seeds'] * 5 != info['count']:
            raise ValueError('Cannot reproduce this suite: unexpected scenario count.')
    elif (source / 'test_metadata.json').is_file():
        kind = 'tests'
        info = json.loads((source / 'test_metadata.json').read_text(encoding='utf-8'))
        required = ('junit.xml', 'pytest_stdout.txt', 'pytest_stderr.txt')
        identity = {'tests': info.get('counts', {}).get('tests'), 'exit_code': info.get('exit_code')}
    else:
        raise ValueError('Expected summary.json (run), aggregate.json (suite) or test_metadata.json (tests).')
    missing = [f for f in required if not (source / f).is_file()]
    if missing:
        raise ValueError(f'Missing experiment provenance/outputs: {missing}. Generate a fresh run with this version.')
    backend = 'tests' if kind == 'tests' else info.get('backend')
    if backend not in ('kinematic', 'mujoco', 'tests'):
        raise ValueError(f'Unsupported backend: {backend!r}')
    dest = ROOT / 'results' / backend / name
    if dest.exists():
        raise FileExistsError(f'Archive already exists (no overwrite): {dest}')
    if source == dest or dest.is_relative_to(source) or source.is_relative_to(dest):
        raise ValueError('Source and destination cannot contain each other.')
    files = [p for p in sorted(source.rglob('*')) if p.is_file()]
    if not files:
        raise ValueError('Nothing to archive')
    for f in files:
        if f.is_symlink():
            raise ValueError(f'Symlinks are not archived: {f}')
        if f.stat().st_size > SIZE_LIMIT_BYTES and not allow_large:
            raise ValueError(f'File is >49 MiB: {f}. Compress video or use --allow-large after configuring Git LFS.')
    # Snapshot selection, never move/delete the source output directory.
    dest.mkdir(parents=True)
    try:
        hashed = []
        for f in files:
            relative = f.relative_to(source)
            target = dest / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, target)
            hashed.append({'path': relative.as_posix(), 'bytes': target.stat().st_size, 'sha256': sha256(target)})
        cfgpath = (dest / 'config_snapshot.json').relative_to(ROOT).as_posix() if kind != 'tests' else None
        if kind == 'run':
            command = ('python -m turtlebot_integrated.cli run'
                       f' --backend {backend} --scenario {identity["scenario"]} --seed {identity["seed"]}'
                       f' --config {cfgpath} --out outputs/reproduced_{name}')
            if (dest / 'demo_capture.mp4').is_file():
                command += ' --video'
            runmeta = json.loads((dest / 'run_metadata.json').read_text(encoding='utf-8'))
            source_head = runmeta.get('git_commit_at_run')
        elif kind == 'suite':
            command = ('python -m turtlebot_integrated.cli suite'
                       f' --backend {backend} --seeds {identity["seeds"]}'
                       f' --config {cfgpath} --out outputs/reproduced_{name}')
            suite_meta = json.loads((dest / 'suite_metadata.json').read_text(encoding='utf-8'))
            source_head = suite_meta.get('git_commit_at_run')
            if suite_meta.get('video_requested'):
                command += ' --video'
        else:
            command = f'python scripts/record_tests.py --out outputs/reproduced_{name}'
            source_head = info.get('git_commit_at_run')
        manifest = {
            'schema_version': 1,
            'kind': kind,
            'backend': backend,
            'identity': identity,
            'archived_utc': datetime.now(timezone.utc).isoformat(),
            'git_commit_at_run': source_head,
            'git_commit_at_archive': git_head(),
            'assessment_evidence': False,  # a saved run requires human review before it supports an assessment claim
            'potential_mujoco_evidence': backend == 'mujoco',
            'limitations': ('KINEMATIC SOFTWARE TEST DOUBLE; NOT MUJOCO ASSESSMENT EVIDENCE'
                            if backend == 'kinematic' else
                            'Unit/integration tests, not MuJoCo assessment performance evidence'
                            if backend == 'tests' else
                            'MuJoCo output; review scene, sensors and rendering validation independently.'),
            'reproduction_command': command,
            'file_inventory': hashed,
        }
        (dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    except Exception:
        shutil.rmtree(dest)
        raise
    return dest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Path inside outputs/ containing summary.json or aggregate.json')
    parser.add_argument('--name', required=True, help='Unique archive name (never reused)')
    parser.add_argument('--allow-large', action='store_true', help='Only when Git LFS has been configured for large videos')
    args = parser.parse_args()
    print(f'Archived: {archive(args.source, args.name, args.allow_large).relative_to(ROOT)}')
    print('To keep it permanently on GitHub: git add results/ && git commit -m "Archive experiment" && git push')


if __name__ == '__main__':
    main()
