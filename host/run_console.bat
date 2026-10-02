@echo off
rem 宠物环传感器控制台（正式版）启动器 —— 使用 WorkBuddy 托管 venv（含 bleak/PySide6/pyqtgraph）
set VENV_PY=C:\Users\pc\.workbuddy\binaries\python\envs\default\Scripts\python.exe
cd /d %~dp0
"%VENV_PY%" -m petring_console
