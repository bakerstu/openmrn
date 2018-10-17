
REGION_ALIAS("REGION_RESET", flash_boot);
REGION_ALIAS("REGION_INT", flash_boot);
REGION_ALIAS("REGION_PGM", flash_user);

/* must be on a 4kb page boundary; rounded down */
_ebase_address  = ORIGIN(flash_boot);
_textoffs = ORIGIN(REGION_PGM) + 0;

SECTIONS
{
  /*
  ** PIC32MX configuration registers
  */
  .PIC32MX_cfg_init :
  {
    KEEP(*(.PIC32MX_cfg_init))
  } >flash_cfg /*AT>PIC32MX_physical_flash_cfg*/
  /* Assert exactly 4 words are placed in configuration area */
  /* Note: terminating semicolon causes syntax error for ASSERT in following line ! */

  ASSERT (SIZEOF(.PIC32MX_cfg_init) == 0x10, "PIC32MX configuration register set must be 4 words long")
	
}
