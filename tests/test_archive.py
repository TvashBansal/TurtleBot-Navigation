"""Test permanent archive integrity without pretending the kinematic backend is MuJoCo."""
import json
from pathlib import Path

import pytest

from scripts import archive_run, verify_archive


def _fake_run(source: Path):
    source.mkdir(parents=True)
    (source / 'summary.json').write_text(json.dumps({'backend':'kinematic','scenario':'nominal','seed':7}))
    (source / 'config_snapshot.json').write_text('{}')
    (source / 'run_metadata.json').write_text(json.dumps({'git_commit_at_run':None}))
    (source / 'trajectory.json').write_text('[]')


def test_archive_copies_all_files_and_checks_hashes(tmp_path, monkeypatch):
    monkeypatch.setattr(archive_run,'ROOT',tmp_path)
    src = tmp_path / 'outputs' / 'nominal'
    _fake_run(src)
    result = archive_run.archive(src, 'first_run')
    assert result == tmp_path / 'results' / 'kinematic' / 'first_run'
    assert verify_archive.verify(result / 'manifest.json') == []
    manifest = json.loads((result / 'manifest.json').read_text())
    assert not manifest['assessment_evidence']
    assert '--seed 7' in manifest['reproduction_command']
    (result / 'trajectory.json').write_text('[1]')
    assert any('changed trajectory.json' in item for item in verify_archive.verify(result / 'manifest.json'))


def test_archive_refuses_to_overwrite(tmp_path, monkeypatch):
    monkeypatch.setattr(archive_run,'ROOT',tmp_path)
    src = tmp_path / 'outputs' / 'nominal'
    _fake_run(src)
    archive_run.archive(src, 'existing')
    with pytest.raises(FileExistsError):
        archive_run.archive(src, 'existing')


def test_archive_validates_name(tmp_path):
    with pytest.raises(ValueError):
        archive_run.archive(tmp_path, '../escape')
