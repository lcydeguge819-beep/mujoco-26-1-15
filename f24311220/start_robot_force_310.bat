@echo off
setlocal

echo ==========================================
echo    Robot Launcher (Force Python 3.10)
echo ==========================================
echo.

:: Check if py launcher exists
where py >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] 'py' launcher not found.
    echo Please reinstall Python 3.10 and check "Install launcher for all users".
    pause
    exit /b
)

:: Force use of Python 3.10
echo [INFO] Found Python 3.10. Initializing...
set PY_CMD=py -3.10

:: Verify it works
%PY_CMD% --version
if %errorlevel% neq 0 (
    echo [ERROR] Could not launch Python 3.10.
    echo Please make sure Python 3.10 is correctly installed.
    pause
    exit /b
)

:: Install Dependencies using 3.10
echo.
echo [INFO] Installing/Updating libraries for Python 3.10...
%PY_CMD% -m pip install --upgrade mujoco numpy glfw keyboard opencv-python

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Installation failed.
    pause
    exit /b
)

:: Run the script
echo.
echo [INFO] Starting Robot Simulation...
%PY_CMD% run_vacuum.py

echo.
if %errorlevel% neq 0 (
    echo [ERROR] Simulation exited with error code %errorlevel%.
)
echo [INFO] Process finished.
pause
