@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 请先停止通信并关闭这三份 FreeMASTER 工程。
echo 本工具只更新分组及 ELF 地址，不烧录、不连接、不发送电机命令。
echo 1 底盘悬空调参（同固件抬臂保持）
echo 2 Yaw 速度环调参（其它轴失能）
echo 3 整车慢速与 Yaw 位置环
choice /c 123 /n /m "选择 1 / 2 / 3："
if errorlevel 3 goto vehicle
if errorlevel 2 goto yaw
python "_工具\生成工程.py" --profile ChassisBenchDebug
goto done
:yaw
python "_工具\生成工程.py" --profile YawBenchDebug
goto done
:vehicle
python "_工具\生成工程.py" --profile VehicleDebug
:done
if errorlevel 1 echo 未成功生成，请按报错检查 Python 依赖及对应固件 ELF。
pause
