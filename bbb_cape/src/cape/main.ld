MEMORY
{
  FLASH (rx) : ORIGIN = 0x00008000, LENGTH = 0x38000   /* 256k - 32k */
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x08000   /*  32k */
}

SECTIONS
{
	.text :
	{
		KEEP(*(Reset_Handler))
	} > FLASH
}
