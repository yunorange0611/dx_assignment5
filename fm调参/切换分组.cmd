@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 请先停止通信并关闭这三份 FreeMASTER 工程。
echo 日常只有一份 Debug 整车固件；本工具只刷新分组及 ELF 地址。
echo 不烧录、不连接、不发送电机命令。选调参对象请在 FreeMASTER 改 robot_tune_mode。
python "_工具\生成工程.py"
if errorlevel 1 echo 未成功生成，请按报错检查 Python 依赖及 Debug 固件 ELF。
pause
