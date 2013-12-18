MEMORY
{
  FLASH (rx) : ORIGIN = 16k, LENGTH = 1024k - 16k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128k
}

SECTIONS
{
	.text :
	{
		/* make sure this stays at the beginning of FLASH */
		KEEP(*(Reset_Handler))
	} > FLASH
}
