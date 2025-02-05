ENTRY(xpatch_init)

/*
    ITCMRAM (rwx)   : ORIGIN = 0x00000000, LENGTH = 0x00010000
    DTCMRAM (wx)    : org = 0x20010000, len = 64k             
*/

MEMORY
{
    /*
    originally 
        24 kB for program
        64 kB for variables
    new:
        44+8 = 52 kB for program
        64 kB for variables 
    next:
        44+8+16 = 68 kB for program?? (sram3?)
    */

    /* for 755 bodge it all into AHB SRAM, 0x30000000 for the moment, we need to look at what isn't and is cached for DMA  */

    CODE  (rwx): ORIGIN = 0x00000000, LENGTH = 0x00010000 /* 64kB   */
    SRAM1 (wx) : org = 0x24040000, len = 256k      /* AXI SRAM leave 256 for patcher, was 512*/
    SRAM2 (wx) : org = 0x30000000, len = 288k      /* AHB SRAM1+SRAM2+SRAM3 */
    SRAM3 (wx) : org = 0x38000000, len = 64k       /* AHB SRAM4 */
    DATA (rw)  : ORIGIN = 0x20010000, LENGTH = 0x00010000 /* CCM - ccmramend */
    SDRAM        : ORIGIN = 0xC0000000, LENGTH = 0x02000000 /* 32 MB */
}

SECTIONS
{

    .text : ALIGN(16) SUBALIGN(16)
    {
    /*
    Note: the .boot section _must_ be first in the resulting binary,
    as that is where the firmware calls the patch when loaded.
    */
        * (.boot);
        * (.text);
        etext = .;
    } > CODE
    
    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__ctor_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__ctor_array_end = .);
    } > CODE

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__dtor_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__dtor_array_end = .);
    } > CODE

    .rodata : ALIGN(4) SUBALIGN(4) 
    {
        * (.rodata);
        * (.rodata.*);
    } > CODE

    .data (NOLOAD): ALIGN(4) SUBALIGN(4)
    {
        sdata = .;
        * (.data);
        edata = .;
    } > CODE

    .sram1 (NOLOAD): ALIGN(4) SUBALIGN(4) 
    {
        * (.sram1);
        * (.sram1.*);
    } > SRAM1

    .sram2 (NOLOAD): ALIGN(4) SUBALIGN(4) 
    {
        * (.sram2);
        * (.sram2.*);
    } > SRAM2

    .sram3 (NOLOAD): ALIGN(4) SUBALIGN(4) 
    {
        * (.sram3);
        * (.sram3.*);
    } > SRAM3
   
    .bss (NOLOAD): ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(_pbss_start = .);        
        sbss = .;
        * (.bss);
        ebss = .;
        PROVIDE(_pbss_end = .);        
    } > DATA
    
    .sdram (NOLOAD) : ALIGN(4) SUBALIGN(4)
    {
        KEEP(*(.sdram))
        KEEP(*(.sdram.*))
        PROVIDE(_sdram_dyn_start = .);
        PROVIDE(_sdram_dyn_end = ORIGIN(SDRAM) + LENGTH(SDRAM));
    } > SDRAM


 /*   
    .ARM.exidx : 
    {
       __exidx_start = .;
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
       __exidx_end = .;
    } >CCMSRAM
    _etext = .;
*/
}
