"""Run the real tuning owner with test-only confirmed configuration in ARM/Unicorn.

Generated configuration copies live under build/tests_runtime_tuning; production
confirmation flags are never changed. This tests control logic, not real hardware.
"""
import pathlib
import re
import struct
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / "build/tests_runtime_tuning"
BUILD.mkdir(parents=True, exist_ok=True)
sys.path.insert(0, str(ROOT / "build/test_deps"))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_SP


def confirmed_copy(source, target, changes):
    data = (ROOT / source).read_text(encoding="utf-8")
    for pattern, replacement in changes:
        data, count = re.subn(pattern, replacement, data, count=1)
        if count != 1:
            raise SystemExit(f"Test configuration layout changed: {source}: {pattern}")
    path = BUILD / target
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(data, encoding="utf-8")
    return path


arm_config = confirmed_copy("2User_Driver/Arm/arm_config.cpp", "arm_config.cpp", [
    (r"\n    false,", "\n    true,"),
    (r"\n        false,", "\n        true,"),
])
operator_config = confirmed_copy("2User_Driver/Remote/operator_config.cpp", "operator_config.cpp", [
    (r"(const OperatorConfig kOperatorConfig = \{\s*)false,", r"\g<1>true,"),
])
confirmed_copy("2User_Driver/Arm/yaw_config.h", "include/Arm/yaw_config.h", [
    (r"kYawSafetyConfirmed = false;", "kYawSafetyConfirmed = true;"),
])
confirmed_copy("2User_Driver/Imu/attitude_estimator.h", "include/Imu/attitude_estimator.h", [
    (r"installation_confirmed = false, thresholds_confirmed = false;",
     "installation_confirmed = true, thresholds_confirmed = true;"),
])

includes = [BUILD / "include"] + [ROOT / item for item in [
    "3Task", "1Bsp", "Core/Inc", "2User_Driver", "Drivers/STM32F4xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Include", "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
    "Middlewares/Third_Party/FreeRTOS/Source/include",
    "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
    "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
]]
flags = ["-O1", "-g", "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft",
         "-ffunction-sections", "-fdata-sections", "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
         "-DARM_FOUR_AXIS_BENCH=0", "-DVEHICLE_SAFETY_ENABLED=1"]
for path in includes:
    flags.extend(["-I", str(path)])
dr16 = BUILD / "dr16_bsp.o"
subprocess.run(["arm-none-eabi-gcc", "-std=c11", *flags, "-c", str(ROOT / "1Bsp/dr16_bsp.c"),
                "-o", str(dr16)], check=True)
sources = [ROOT / item for item in [
    "tests/test_runtime_tuning.cpp", "tests/test_can_tx.cpp", "1Bsp/CAN_bsp.cpp",
    "2User_Driver/Arm/arm_control.cpp", "2User_Driver/Remote/operator_control.cpp",
    "2User_Driver/Vehicle/robot_tuning.cpp", "3Task/robot_control.cpp",
    "3Task/task_protect.cpp", "3Task/yaw_service.cpp",
]] + [arm_config, operator_config, dr16]
elf = BUILD / "test_runtime_tuning.elf"
subprocess.run(["arm-none-eabi-g++", "-std=c++17", *flags, "-fno-exceptions", "-fno-rtti",
                "-fno-threadsafe-statics", "-fno-builtin", "-nostartfiles", "-Wl,--gc-sections",
                "-T", str(ROOT / "tests/test.ld"), *map(str, sources),
                "-lm", "-lc", "-lgcc", "-o", str(elf)], check=True)
emu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
emu.mem_map(0x10000, 0x400000)
with elf.open("rb") as stream:
    obj = ELFFile(stream)
    for segment in obj.iter_segments():
        if segment["p_type"] == "PT_LOAD":
            emu.mem_write(segment["p_vaddr"], segment.data())
    symbols = {s.name: s["st_value"] for s in obj.get_section_by_name(".symtab").iter_symbols()}
    entry = obj["e_entry"]
emu.reg_write(UC_ARM_REG_SP, 0x40f000)
emu.hook_add(UC_HOOK_INTR, lambda uc, number, user: uc.emu_stop())
emu.emu_start(entry | 1, 0, timeout=30_000_000, count=100_000_000)
read = lambda name: struct.unpack("<I", emu.mem_read(symbols[name], 4))[0]
checks, failed, done = read("test_checks"), read("test_failed_line"), read("test_done")
if not done or failed:
    raise SystemExit(f"FAIL runtime owner: line={failed}, done={done}, assertions={checks}")
print(f"PASS [Runtime tuning owner]: {checks} assertions; test-only confirmed config; no hardware validation")
