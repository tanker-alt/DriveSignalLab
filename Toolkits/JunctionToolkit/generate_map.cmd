@echo off
pushd "%~dp0"
python -B wizard.py %*
set "wizard_result=%errorlevel%"
popd
exit /b %wizard_result%
