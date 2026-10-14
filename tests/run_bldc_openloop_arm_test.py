"""Run the real open-loop BLDC driver in an ARM emulator.

Requires arm-none-eabi-gcc, unicorn and pyelftools. Peripheral registers are
memory-mapped but timer progression and the physical motor are not simulated.
"""
from pathlib import Path
import shutil
import subprocess

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

project = Path(__file__).resolve().parents[1]
output = project / "build" / "bldc-openloop-tests"
output.mkdir(parents=True, exist_ok=True)
elf_path = output / "bldc_openloop_test.elf"
compiler = shutil.which("arm-none-eabi-gcc")
if compiler is None:
    raise SystemExit("arm-none-eabi-gcc must be on PATH")
includes = ["Core/APP", "Core/BSP", "Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
            "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include"]
command = [compiler, "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-O1", "-g",
           "-Wall", "-Wextra", "-Werror", "-fno-builtin", "-nostartfiles", "-nostdlib",
           "-DSTM32F405xx", "-DUSE_HAL_DRIVER",
           "-DBLDC_CONTROL_MODE=BLDC_CONTROL_MODE_OPEN_LOOP_1637"]
command += [f"-I{project / path}" for path in includes]
command += [str(project / path) for path in ["Core/BSP/BLDC_OpenLoop1637.c",
                                              "Core/APP/sbus_motor_control.c",
                                              "tests/bldc_openloop_arm_test.c"]]
command += [f"-T{project / 'tests/bldc_test.ld'}", "-lgcc", "-o", str(elf_path)]
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
    raise SystemExit(f"Open-loop driver regression failed at tests/bldc_openloop_arm_test.c:{result}")
print("PASS: precharge/alignment, 30 us blanking, sequential six-step commutation,")
print("      uninterrupted 20 kHz PWM counter, target ramp and emergency stop.")
print("      CH3 requests drive real period changes, acceleration/deceleration and duty-zero stop.")
