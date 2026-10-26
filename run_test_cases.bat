@echo off
cd /d "%~dp0"
python evaluate_test_cases.py --group all
set RESULT=%ERRORLEVEL%
echo.
echo 原始 1V1/1V2/1V3 场景回归结束，退出码：%RESULT%
pause
exit /b %RESULT%
