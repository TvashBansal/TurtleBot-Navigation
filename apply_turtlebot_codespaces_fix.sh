#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ ! -f pyproject.toml || ! -d .git ]]; then
  echo "ERROR: Upload this script into the root of your TurtleBot-Navigation Git repository." >&2
  exit 1
fi
mkdir -p .devcontainer
cat > ".devcontainer/Dockerfile" <<'TURTLEBOT_DEVCONTAINER_DOCKERFILE'
# Debian/glibc is required so pip can install prebuilt MuJoCo and OpenCV wheels.
# The previous session was downloading musllinux packages and compiling MuJoCo.
FROM mcr.microsoft.com/devcontainers/python:1-3.11-bookworm

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       libosmesa6 libosmesa6-dev libgl1 libgl1-mesa-dri \
       libglfw3 libglew2.2 libegl1 ffmpeg \
    && rm -rf /var/lib/apt/lists/*

ENV MUJOCO_GL=osmesa \
    MPLBACKEND=Agg \
    LIBGL_ALWAYS_SOFTWARE=1
TURTLEBOT_DEVCONTAINER_DOCKERFILE
cat > ".devcontainer/devcontainer.json" <<'TURTLEBOT_DEVCONTAINER_DEVCONTAINER_JSON'
{
  "name": "TurtleBot-Navigation - MuJoCo (Debian Python 3.11)",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "postCreateCommand": "bash .devcontainer/setup.sh",
  "remoteEnv": {
    "MUJOCO_GL": "osmesa",
    "MPLBACKEND": "Agg",
    "LIBGL_ALWAYS_SOFTWARE": "1"
  },
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.vscode-pylance"
      ]
    }
  }
}
TURTLEBOT_DEVCONTAINER_DEVCONTAINER_JSON
cat > ".devcontainer/setup.sh" <<'TURTLEBOT_DEVCONTAINER_SETUP_SH'
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
TURTLEBOT_DEVCONTAINER_SETUP_SH
chmod +x .devcontainer/setup.sh
echo "Environment configuration restored:"
ls -l .devcontainer/Dockerfile .devcontainer/devcontainer.json .devcontainer/setup.sh
echo
echo "Important: this does NOT change your current Alpine environment."
echo "Next commit these files and create a NEW Codespace from main."
git status --short -- .devcontainer
