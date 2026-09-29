ENTRY(test_main)
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1M
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}
SECTIONS
{
    .text : { *(.text*) *(.rodata*) *(.ARM.extab*) *(.ARM.exidx*) } > FLASH
    .data : { *(.data*) } > RAM
    .bss : { *(.bss*) *(COMMON) } > RAM
}
