/*
** ###################################################################
**     Processor:           TW9001 with 64 KB SRAM
**     Compiler:            GNU C Compiler
**
**     Abstract:
**         Linker file for the GNU C Compiler
**
** ###################################################################
*/

/* Entry Point */
ENTRY(Reset_Handler)

HEAP_SIZE  = DEFINED(__heap_size__)  ? __heap_size__  : 0x00000400;
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x00000400;

/* Specify the memory areas */
MEMORY
{
  FLASH	(RX)  : ORIGIN = 0x00000000, LENGTH = 0x00040000	/* Flash 256K */
  RAM		(RW)  : ORIGIN = 0x20000000, LENGTH = 0x00010000	/* RAM   64K  */
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into internal flash */
  .interrupts :
  {
    __VECTOR_TABLE = .;
    __interrupts_start__ = .;
    . = ALIGN(4);
    KEEP(*(.isr_vector))     /* Startup code */
    __interrupts_end__ = .;
    . = ALIGN(4);
  } > FLASH

  /* The program code and other data goes into internal flash */
  .text :
  {
    . = ALIGN(4);
    *(.text)                 /* .text sections (code) */
    *(.text*)                /* .text* sections (code) */
    *(.rodata)               /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)              /* .rodata* sections (constants, strings, etc.) */
    *(.init)                 /* section used in crti.o files */
    *(.fini)                 /* section used in crti.o files */
    *(.eh_frame)             /* section used in crtbegin.o files */
    . = ALIGN(4);
  } > FLASH

  /* Section used by the libgcc.a library for fvp4 */
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } > FLASH

  __etext = .;    /* Define a global symbol at end of code. */
  __DATA_ROM = .; /* Symbol is used by startup for data initialization. */

  __RAM_START = ORIGIN(RAM);

  .data : AT(__DATA_ROM)
  {
    . = ALIGN(4);
    __DATA_RAM = .;
    __data_start__ = .;      /* Create a global symbol at data start. */
    *(.data)                 /* .data sections */
    *(.data*)                /* .data* sections */
    . = ALIGN(4);
    __data_end__ = .;        /* Define a global symbol at data end. */
  } > RAM

  __DATA_END = __DATA_ROM + (__data_end__ - __data_start__);
  __CODE_ROM = __DATA_END; /* Symbol is used by code initialization. */

  .code : AT(__CODE_ROM)
  {
    . = ALIGN(4);
    __CODE_RAM = .;
    __code_start__ = .;      /* Create a global symbol at code start. */
    __code_ram_start__ = .;
    *(.code_ram)             /* Custom section for storing code in RAM */
    . = ALIGN(4);
    __code_end__ = .;        /* Define a global symbol at code end. */
    __code_ram_end__ = .;
  } > RAM

  __CODE_END = __CODE_ROM + (__code_end__ - __code_start__);
  __CUSTOM_ROM = __CODE_END;
  __customSection_start__ = __CODE_END;
  __CUSTOM_END = __CODE_END;
  __rom_end    = __CODE_END;

  /* Uninitialized data section. */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section. */
    . = ALIGN(4);
    __BSS_START = .;
    __bss_start__ = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    __BSS_END = .;
  } > RAM

   /* Put heap section after the program data */
  .heap :
  {
    . = ALIGN(8);
    __end__ = .;
    __heap_start__ = .;
    PROVIDE(end = .);
    PROVIDE(_end = .);
    PROVIDE(__end = .);
    __HeapBase = .;
    . += HEAP_SIZE;
    __HeapLimit = .;
    __heap_limit = .;
    __heap_end__ = .;
  } > RAM

  /* Initializes stack on the end of block */
  __StackTop   = ORIGIN(RAM) + LENGTH(RAM);
  __StackLimit = __StackTop - STACK_SIZE;
  PROVIDE(__stack = __StackTop);
  __RAM_END = __StackTop;

  .stack __StackLimit :
  {
    . = ALIGN(8);
    __stack_start__ = .;
    . += STACK_SIZE;
    __stack_end__ = .;
  } > RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
  
  /* Memory validation */
  ASSERT(__rom_end <= (ORIGIN(FLASH) + LENGTH(FLASH)), "region FLASH overflowed!")

  ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack and heap")
}

