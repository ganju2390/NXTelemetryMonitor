@echo off
cd /d "%~dp0"
where uv >nul 2>nul
if %errorlevel% equ 0 (
    uv run python -m groundstation %*
) else if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -m groundstation %*
) else (
    echo Install uv and run: uv sync --locked
    pause
)
