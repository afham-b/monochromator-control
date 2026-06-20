#!/bin/zsh

# Simple macOS launcher for the Monochromator GUI.
# This intentionally mirrors the manual workflow:
#   cd into this project
#   activate venv
#   python3 monochromator_gui.py

set -u

SCRIPT_PATH="${0:A}"
PROJECT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
VENV_ACTIVATE="$PROJECT_DIR/venv/bin/activate"
GUI_SCRIPT="$PROJECT_DIR/monochromator_gui.py"
LAUNCH_TTY="$(tty)"

close_this_terminal_window() {
  local tty_path="$1"

  # Delay the close until after this shell has exited, otherwise Terminal may
  # think the launcher is still running and ask for confirmation.
  {
    sleep 0.4
    /usr/bin/osascript <<OSA >/dev/null 2>&1
tell application "Terminal"
  set targetWindow to missing value
  repeat with w in windows
    repeat with t in tabs of w
      if tty of t is "$tty_path" then
        set targetWindow to w
        exit repeat
      end if
    end repeat
    if targetWindow is not missing value then exit repeat
  end repeat
  if targetWindow is not missing value then close targetWindow
end tell
OSA
  } >/dev/null 2>&1 &
}

cd "$PROJECT_DIR" || {
  echo "Could not cd into project directory: $PROJECT_DIR"
  echo "Press any key to close this window..."
  read -k 1
  exit 1
}

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "Could not find venv activation script: $VENV_ACTIVATE"
  echo "Expected to run exactly like: source venv/bin/activate"
  echo "Press any key to close this window..."
  read -k 1
  exit 1
fi

if [[ ! -f "$GUI_SCRIPT" ]]; then
  echo "Could not find GUI script: $GUI_SCRIPT"
  echo "Press any key to close this window..."
  read -k 1
  exit 1
fi

echo "Launching Monochromator GUI from: $PROJECT_DIR"
echo "Activating venv..."
source "$VENV_ACTIVATE"

echo "Python: $(command -v python3)"
echo "Starting GUI..."
echo ""

python3 "$GUI_SCRIPT"
STATUS=$?

echo ""
echo "Monochromator GUI exited with status $STATUS."

if [[ "$STATUS" -ne 0 ]]; then
  echo "Press any key to close this window..."
  read -k 1
  exit "$STATUS"
fi

echo "Closing launcher terminal..."
close_this_terminal_window "$LAUNCH_TTY"
exit 0
