"""Check all Git-tracked experiment snapshots against their SHA-256 manifest."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
try:
    from .archive_run import sha256
except ImportError:  # executed directly as python scripts/verify_archive.py
    from archive_run import sha256

ROOT = Path(__file__).resolve().parents[1]


def verify(path: Path) -> list[str]:
    failures = []
    archive = path.parent
    try:
        manifest = json.loads(path.read_text(encoding='utf-8'))
    except Exception as e:
        return [f'{path}: unreadable manifest ({e})']
    for entry in manifest['file_inventory']:
        relative = Path(entry['path'])
        if relative.is_absolute() or '..' in relative.parts:
            failures.append(f'{path}: unsafe path {entry["path"]}')
            continue
        f = archive / relative
        if not f.is_file():
            failures.append(f'{path}: missing {entry["path"]}')
        elif f.stat().st_size != entry['bytes'] or sha256(f) != entry['sha256']:
            failures.append(f'{path}: changed {entry["path"]}')
    return failures


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', nargs='?', type=Path, default=ROOT / 'results')
    args = parser.parse_args()
    manifests = sorted(args.folder.rglob('manifest.json'))
    if not manifests:
        parser.error(f'No manifest.json files found in {args.folder}')
    failures = [err for path in manifests for err in verify(path)]
    for bad in failures:
        print('FAIL:', bad)
    print(f'Validated {len(manifests)} archived experiment(s); {len(failures)} issue(s).')
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
