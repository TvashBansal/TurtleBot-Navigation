"""Save pytest JUnit XML, console logs and a provenance record for later Git archival."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def run_tests(out: Path) -> dict:
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).isoformat()
    head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True, check=False)
    cmd = [sys.executable, '-m', 'pytest', '-q', f'--junitxml={out / "junit.xml"}']
    completed = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, check=False)
    (out / 'pytest_stdout.txt').write_text(completed.stdout, encoding='utf-8')
    (out / 'pytest_stderr.txt').write_text(completed.stderr, encoding='utf-8')
    counts = {'tests': 0, 'failures': 0, 'errors': 0, 'skipped': 0}
    if (out / 'junit.xml').is_file():
        parsed = ET.parse(out / 'junit.xml').getroot()
        # pytest usually gives <testsuites><testsuite .../></testsuites>
        suites = [parsed] if parsed.tag == 'testsuite' else parsed.findall('testsuite')
        for suite in suites:
            for key in counts:
                counts[key] += int(suite.attrib.get(key, 0))
    try:
        pytest_version = importlib.metadata.version('pytest')
    except importlib.metadata.PackageNotFoundError:
        pytest_version = None
    report = {
        'kind': 'python_tests',
        'created_utc': stamp,
        'git_commit_at_run': head.stdout.strip() if head.returncode == 0 else None,
        'python': sys.version.split()[0],
        'pytest': pytest_version,
        'platform': platform.platform(),
        'command': 'python -m pytest -q --junitxml=<recorded_output>/junit.xml',
        'exit_code': completed.returncode,
        'counts': counts,
        'limitation': 'Unit and kinematic integration checks, not a physical or MuJoCo run',
    }
    (out / 'test_metadata.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('outputs/test_reports'))
    args = parser.parse_args()
    report = run_tests(args.out)
    print(json.dumps(report, indent=2))
    if report['exit_code']:
        raise SystemExit(report['exit_code'])


if __name__ == '__main__':
    main()
