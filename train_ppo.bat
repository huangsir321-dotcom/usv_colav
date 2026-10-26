@echo off
chcp 65001 >nul
cd /d "%~dp0"
python train_ppo.py --timesteps 300000 --n-envs 4
pause
