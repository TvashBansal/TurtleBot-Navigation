#!/usr/bin/env bash
# Minimal recovery configuration for the TurtleBot-Navigation GitHub Codespace.
# Does not modify source code, results, or installed packages.
set -euo pipefail

if [[ ! -d .git && ! -f .git ]]; then
  echo "ERROR: Run this script from the root of the TurtleBot-Navigation Git repository." >&2
  exit 1
fi

mkdir -p .devcontainer
cat > .devcontainer/devcontainer.json <<'JSON'
{
  "name": "TurtleBot-Navigation - Recovery (Debian Python 3.11)",
  "image": "mcr.microsoft.com/devcontainers/python:1-3.11-bookworm",
  "customizations": {
    "vscode": {
      "extensions": [
        "ms-python.python",
        "ms-python.vscode-pylance"
      ]
    }
  }
}
JSON

printf '%s\n' 'Updated .devcontainer/devcontainer.json to use a standard Debian Python 3.11 image.'
printf '%s\n' 'No Dockerfile or postCreateCommand is used in this configuration.'
printf '%s\n' 'Next: git add .devcontainer/devcontainer.json && git commit -m "Repair Codespaces configuration" && git push origin main'
printf '%s\n' 'Then rebuild the container from the Codespaces Command Palette.'
