@echo off
setlocal
if exist "%~dp0..\..\generator-env\Scripts\python.exe" (
  "%~dp0..\..\generator-env\Scripts\python.exe" -B "%~dp0wizard.py" %*
) else (
  python -B "%~dp0wizard.py" %*
)
exit /b %errorlevel%
