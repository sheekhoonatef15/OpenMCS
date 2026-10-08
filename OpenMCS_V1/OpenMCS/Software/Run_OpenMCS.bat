@echo off
setlocal
cd /d "%~dp0"

where py >nul 2>nul
if errorlevel 1 goto use_python
py -3 -c "import numpy, serial, PyQt6, pyqtgraph" >nul 2>nul
if errorlevel 1 goto missing_py
py -3 OpenMCS_GUI.py
exit /b %errorlevel%

:use_python
where python >nul 2>nul
if errorlevel 1 goto no_python
python -c "import numpy, serial, PyQt6, pyqtgraph" >nul 2>nul
if errorlevel 1 goto missing_python
python OpenMCS_GUI.py
exit /b %errorlevel%

:missing_py
echo OpenMCS Python packages are missing.
echo Run: py -3 -m pip install -r "%~dp0requirements.txt"
pause
exit /b 1

:missing_python
echo OpenMCS Python packages are missing.
echo Run: python -m pip install -r "%~dp0requirements.txt"
pause
exit /b 1

:no_python
echo Python 3.10 or newer was not found.
pause
exit /b 1
