#!/usr/bin/env bash

# Simple Linux launcher for the Monochromator GUI.
# This mirrors the manual workflow:
#   cd into this project
#   activate venv
#   python3 monochromator_gui.py

set -u

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_ACTIVATE="$PROJECT_DIR/venv/bin/activate"
GUI_SCRIPT="$PROJECT_DIR/monochromator_gui.py"

cd "$PROJECT_DIR" || {
  echo "Could not cd into project directory: $PROJECT_DIR"
  echo "Press Enter to close this window..."
  read -r _
  exit 1
}

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "Could not find venv activation script: $VENV_ACTIVATE"
  echo "Expected Linux venv layout: venv/bin/activate"
  echo "Press Enter to close this window..."
  read -r _
  exit 1
fi

if [[ ! -f "$GUI_SCRIPT" ]]; then
  echo "Could not find GUI script: $GUI_SCRIPT"
  echo "Press Enter to close this window..."
  read -r _
  exit 1
fi

echo "Launching Monochromator GUI from: $PROJECT_DIR"
echo "Activating venv..."
# shellcheck source=/dev/null
source "$VENV_ACTIVATE"

echo "Python: $(command -v python3)"
echo "Starting GUI..."
echo ""

python3 "$GUI_SCRIPT"
STATUS=$?

echo ""
echo "Monochromator GUI exited with status $STATUS."

if [[ "$STATUS" -ne 0 ]]; then
  echo "Press Enter to close this window..."
  read -r _
fi

exit "$STATUS"
