@echo off
chcp 65001 >nul
cd /d "%~dp0"
python stress_test.py --episodes-per-case 2 --max-steps 120
pause
