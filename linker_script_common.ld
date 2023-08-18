/*!****************************************************************************
 * @file
 * linker_script_common.ld
 *
 * @brief
 * Common section layout definitions across all controller types
 *
 * @date  18.08.2023
 ******************************************************************************/

/* Entry Point assignment                                                     */
ENTRY(_start)

/* Sections definition                                                        */
SECTIONS
{
  /* Interrupt Vector Table at start of app memory        */
  .isr_vector :
  {
    . = ALIGN(4);

    KEEP( *(.isr_vector))
    KEEP( *(.isr_vector*))

    . = ALIGN(64);
  } >FLASH

  /* Code and read-only constants in FLASH memory         */
  .text :
  {
    . = ALIGN(4);

    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    
    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);

    *(.rodata)
    *(.rodata*)

    . = ALIGN(4);
  } >FLASH

  /* LibC init, constructors and destructors              */
  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH
  
  /* Stack unwinding information                          */
  .ARM.extab :
  {
    KEEP(*(.ARM.extab*))
    KEEP(*(.gnu.linkonce.armextab.*))
  } >FLASH
  .ARM :
  {
    __exidx_start = .;
    KEEP(*(.ARM.exidx*))
    __exidx_end = .;
  } >FLASH

  /* Initialised data section                             */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    _edata = .;
  } >RAM AT>FLASH

  /* Initialisation data location in FLASH                */
  _sidata = LOADADDR(.data);

  /* Zero-initialised data section                        */
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;

    *(.bss)
    *(.bss*)
    *(COMMON*)
    
    . = ALIGN(4);
    _ebss = .;
    PROVIDE( end = . );    
    
  } >RAM
  
  /* Stack/Heap collision detection                       */
  .heap (NOLOAD) :
  {
    . = ALIGN(4);
    _sheap = .;
    
    /* Calculate available heap size                      */
    __avail_heap_size = LENGTH(RAM) - __stack_size - (_sheap - ORIGIN(RAM));
    
    . = . + MAX(__min_heap_size, __avail_heap_size);
    . = ALIGN(4);
    _eheap = .;
  } >RAM
  
  /* Stack at the end of RAM                              */
  .stack (NOLOAD) :
  {
    . = ALIGN(4);
    _sstack = .;
    . = . + __stack_size;
    _estack = .;
  } >RAM

  /* Remove standard libraries                            */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}