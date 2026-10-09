/* Direct SWD boot at 0x08000000, no bootloader. Confirm exact board before flash. */
ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1M
  AXIRAM (rw) : ORIGIN = 0x24000000, LENGTH = 320K
  MSPRAM (rw) : ORIGIN = 0x2001E000, LENGTH = 8K
}
_estack = ORIGIN(MSPRAM) + LENGTH(MSPRAM);
SECTIONS {
  .isr_vector : { . = ALIGN(1024); KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab*) } > FLASH
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .data : { . = ALIGN(8); _sdata = .; *(.data*) . = ALIGN(8); _edata = .; } > AXIRAM AT>FLASH
  _sidata = LOADADDR(.data);
  _flash_end = LOADADDR(.data) + SIZEOF(.data);
  .bss (NOLOAD) : { . = ALIGN(8); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(8); _ebss = .; } > AXIRAM
  .stack (NOLOAD) : { _stack_limit = .; . += 8192; } > MSPRAM
  /DISCARD/ : { *(.comment) *(.note*) }
}
ASSERT(_ebss <= ORIGIN(AXIRAM)+LENGTH(AXIRAM), "AXIRAM overflow")
ASSERT(_flash_end <= ORIGIN(FLASH)+LENGTH(FLASH), "FLASH overflow")
ASSERT(SIZEOF(.isr_vector) <= 1024, "Vector table alignment insufficient")
