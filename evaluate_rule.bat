@echo off
chcp 65001 >nul
cd /d "%~dp0"
python evaluate.py --policy rule --episodes 30
pause
