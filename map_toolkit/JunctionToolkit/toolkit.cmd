@echo off
setlocal
if exist "%~dp0..\..\generator-env\Scripts\python.exe" (
  "%~dp0..\..\generator-env\Scripts\python.exe" -B "%~dp0toolkit.py" %*
) else (
  python -B "%~dp0toolkit.py" %*
)
exit /b %errorlevel%
