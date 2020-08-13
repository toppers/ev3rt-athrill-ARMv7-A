
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm","elf32-littlearm") 
OUTPUT_ARCH(arm)
STARTUP(start.o)

PROVIDE(hardware_init_hook = 0);
PROVIDE(software_init_hook = 0);
PROVIDE(software_term_hook = 0);

ENTRY(start);

MEMORY
{
  BOOT_LOADER (rx) : ORIGIN = 0x18000000, LENGTH = 0x00004000 
	INTROM(raxi) :  org = 0x18004000, len = 512k
	INTDATA(wax) : org = 0x18204000, len = 512k
    INTRAM (wax)    : org = 0x20000000, LENGTH = 10M
    INTRAM2 (wax)    : org = 0x20A00000, LENGTH = 10M
}


SECTIONS
{
    .boot :
    {
        KEEP(*(.boot_loader)) 
    } > BOOT_LOADER 

    .text :
    {
        Image$$VECTORS$$Base = .;
        * (RESET)
        Image$$VECTORS$$Limit = .;
        . += 0x00000400;

        __text = . ;
      	__vector_start = .	;
    	PROVIDE (_vector_start = .);
        *(vector)
        start.o(.text*)
        *(.ivt)
        *(.boot_data)
        *(.dcd)
	    . = ALIGN(32);
        *(.text*)
        *(.glue_7t)
        *(.glue_7)
    } >INTROM

	.ARM.extab : 
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} >INTROM

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} >INTROM
	__exidx_end = .;

    _etext = .	;
    PROVIDE (etext = .)	;
    .rodata : 
    { 
        . = ALIGN(4);
        *(.rodata)
        *(.rodata.*)
        *(.data.rel.ro)
        . = ALIGN(4);
        __init_array_start = .;
	*(.ctors)
        *(SORT(.init_array.*))
        *(.init_array)
        __init_array_end = .;
        _fini_array_start = .;
        *(SORT(.fini_array.*))
        *(.fini_array)
        _fini_array_end = .;
    } > INTROM

	_textend = .;

	. = ALIGN(4);

	__idata_start = . ;

	.data	:  AT(__idata_start)
	{
		. = ALIGN(4);
		__data_start = . ;
        	*(.rosdata)
		*(.sdata)
		PROVIDE (__gp = .);
		*(.data)
		. = ALIGN(4);
		_edata	=  . ;
	} > INTDATA

	__idata_end = __idata_start + SIZEOF(.data);
	. = ALIGN(4);
	__srdata_start = . ;
	.sdata	:  AT(__srdata_start)
	{
		. = ALIGN(4);
		__sdata_start = . ;
		. = ALIGN(4);
		_esdata	=  . ;
	} > INTDATA

	__srdata_end = __srdata_start + SIZEOF(.sdata);



	__srdata_start = 0x00200000 ;
	.sdata	:  AT(__srdata_start)
	{
		. = ALIGN(4);
		__sdata_start = . ;
		PROVIDE (__gp = . + 0x8000);
		*(.sdata)
		. = ALIGN(4);
		_esdata	=  . ;
	} > INTRAM

	__srdata_end = __srdata_start + SIZEOF(.sdata);


	.bss	   :
	{
	  . = ALIGN(4);
	  __bss_start = .;
	  *(.bss)
	  *(COMMON)
	  . = ALIGN(4);
	  __bss_end = .;
	} > INTRAM

	.bss_noclr	   :
	{
	  . = ALIGN(4);
	  *(.appheap)
	  *(COMMON)
	  . = ALIGN(4);
	} > INTRAM2


	.tdata ALIGN (4) :
	{
		PROVIDE (__ep = .);
		*(.tbyte)
		*(.tcommon_byte)
		*(.tdata)
		*(.tbss)
		*(.tcommon)
	} > INTRAM	
    .comment 0 : { *(.comment) }

  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to 
     the beginning of the section so we begin them at 0.  */

  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }

  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }

  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }

  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }

  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
