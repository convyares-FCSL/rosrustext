#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEV_WS_DEFAULT="$(cd "${REPO_ROOT}/.." && pwd)/rosrustext_dev_ws"
OUT_FILE="${REPO_ROOT}/rosrustext.code-workspace"

DEV_WS="${DEV_WS:-$DEV_WS_DEFAULT}"

cat > "${OUT_FILE}" <<JSON
{
  "folders": [
    { "name": "rosrustext (library)", "path": "${REPO_ROOT}" },
    { "name": "rosrustext_dev_ws (colcon)", "path": "${DEV_WS}" }
  ],
  "settings": {
    "files.exclude": {
      "**/target": true,
      "**/build": true,
      "**/install": true,
      "**/log": true,
      "**/.cargo": true,
      "**/.venv": true
    },
    "search.exclude": {
      "**/target": true,
      "**/build": true,
      "**/install": true,
      "**/log": true,
      "**/.cargo": true,
      "**/.venv": true
    },
    "rust-analyzer.checkOnSave.command": "clippy",
    "rust-analyzer.cargo.allFeatures": true,
    "editor.formatOnSave": true
  }
}
JSON

echo "Wrote ${OUT_FILE}"
echo "Open it with: code \"${OUT_FILE}\""
