"""ARM tests of real application/driver functions; HAL and RTOS boundaries mocked.
Requires arm-none-eabi-gcc, unicorn and pyelftools (pip install unicorn pyelftools).
Does not simulate scheduler context switches, electrical buses or servo mechanics.
"""
from pathlib import Path
import shutil
import subprocess
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

project = Path(__file__).resolve().parents[1]
output = project / "build" / "app-tests"
output.mkdir(parents=True, exist_ok=True)
elf_path = output / "app_test.elf"
compiler = shutil.which("arm-none-eabi-gcc")
if compiler is None:
    raise SystemExit("arm-none-eabi-gcc must be on PATH")
includes = ["Core/APP", "Core/BSP", "Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
            "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include",
            "Middlewares/Third_Party/FreeRTOS-Kernel/include",
            "Middlewares/Third_Party/FreeRTOS-Kernel/portable/GCC/ARM_CM4F"]
command = [compiler, "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-O1", "-g",
           "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
           "-nostartfiles", "--specs=nano.specs", "--specs=nosys.specs",
           "-DSTM32F405xx", "-DUSE_HAL_DRIVER", "-DAPP_USE_FREERTOS=1"]
command += [f"-I{project / directory}" for directory in includes]
command += [str(project / file) for file in ["tests/app_arm_test.c", "Core/BSP/Servo.c",
                                           "Core/BSP/bsp_delay.c", "Core/APP/roll_control.c"]]
command += [f"-T{project / 'tests/bldc_test.ld'}", "-Wl,--gc-sections", "-lm", "-lc", "-lgcc", "-o", str(elf_path)]
subprocess.run(command, check=True)
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x08000000, 0x100000)
cpu.mem_map(0x20000000, 0x20000)
cpu.mem_map(0x40000000, 0x30000)
with elf_path.open("rb") as stream:
    elf = ELFFile(stream)
    for segment in elf.iter_segments():
        if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
            cpu.mem_write(segment["p_vaddr"], segment.data())
    entry = elf.header["e_entry"]
return_address = 0x080FF000
cpu.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
cpu.reg_write(UC_ARM_REG_LR, return_address | 1)
cpu.emu_start(entry | 1, return_address, count=100000000)
if cpu.reg_read(UC_ARM_REG_PC) != return_address:
    raise SystemExit("Test hung or exceeded instruction budget")
result = cpu.reg_read(UC_ARM_REG_R0)
if result:
    raise SystemExit(f"Application regression failed at tests/app_arm_test.c:{result}")
print("PASS: servo startup/direction/clamps, stale/invalid IMU failsafe, periodic release/wrap/overrun,")
print("      yielding conversion delays, queue-full and UART timeout/error paths, output formatting,")
print("      MS5611 datasheet compensation and OSR commands, signed IMU parsing/attitude and recovery.")
