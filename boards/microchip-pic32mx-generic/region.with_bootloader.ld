
REGION_ALIAS("REGION_RESET", flash_user);
REGION_ALIAS("REGION_INT", flash_user);
REGION_ALIAS("REGION_PGM", flash_user);


/* must be on a 4kb page boundary; rounded down */
_ebase_address = ORIGIN(flash_user);
_textoffs = ORIGIN(flash_user) + 0xA00;

