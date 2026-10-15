#!/usr/bin/env bash
set -euo pipefail

# Run automatically when the Codespaces dev container is created/rebuilt.
cd "$(dirname "$0")/.."

printf 'OS: '
. /etc/os-release
printf '%s\n' "$PRETTY_NAME"

if [[ "$ID" != "debian" && "${ID_LIKE:-}" != *debian* ]]; then
  echo 'ERROR: Wrong container OS. Rebuild the Codespace using .devcontainer/devcontainer.json (Debian required).' >&2
  exit 1
fi

if ! command -v python3.11 >/dev/null 2>&1; then
  echo 'ERROR: Python 3.11 not found; expected the Debian Python 3.11 devcontainer image.' >&2
  exit 1
fi

# .venv is git-ignored and may have been made with the previous musl/Python 3.12 image.
# Recreate the virtual environment; this does not touch tracked project files.
python3.11 -m venv --clear .venv
.venv/bin/python -m pip install --upgrade pip

# Fail rather than silently compiling MuJoCo from source (MUJOCO_PATH error).
.venv/bin/python -m pip install --only-binary=:all: -e '.[test]'

.venv/bin/python - <<'PY'
import platform, sys
import mujoco
print(f'Python: {sys.version.split()[0]}')
print(f'libc: {platform.libc_ver()}')
print(f'MuJoCo: {mujoco.__version__}')
print('Environment dependencies installed. Run .venv/bin/python -m pytest -q to validate project tests.')
PY
