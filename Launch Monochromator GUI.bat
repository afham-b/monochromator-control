@echo off
setlocal

REM Simple Windows launcher for the Monochromator GUI.
REM This mirrors the manual workflow:
REM   cd into this project
REM   activate venv
REM   python monochromator_gui.py

set "PROJECT_DIR=%~dp0"
cd /d "%PROJECT_DIR%" || goto :cd_failed

if not exist "%PROJECT_DIR%venv\Scripts\activate.bat" goto :venv_missing
if not exist "%PROJECT_DIR%monochromator_gui.py" goto :gui_missing

echo Launching Monochromator GUI from: %PROJECT_DIR%
echo Activating venv...
call "%PROJECT_DIR%venv\Scripts\activate.bat"

echo Python:
where python
echo Starting GUI...
echo.

python "%PROJECT_DIR%monochromator_gui.py"
set "STATUS=%ERRORLEVEL%"

echo.
echo Monochromator GUI exited with status %STATUS%.

if not "%STATUS%"=="0" (
    echo Press any key to close this window...
    pause >nul
)

exit /b %STATUS%

:cd_failed
echo Could not cd into project directory: %PROJECT_DIR%
echo Press any key to close this window...
pause >nul
exit /b 1

:venv_missing
echo Could not find venv activation script: %PROJECT_DIR%venv\Scripts\activate.bat
echo Expected Windows venv layout: venv\Scripts\activate.bat
echo Press any key to close this window...
pause >nul
exit /b 1

:gui_missing
echo Could not find GUI script: %PROJECT_DIR%monochromator_gui.py
echo Press any key to close this window...
pause >nul
exit /b 1
