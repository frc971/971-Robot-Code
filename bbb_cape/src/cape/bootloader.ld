MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x08000   /*  32k */
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x08000   /*  32k */
}

SECTIONS
{
	.text :
	{
		KEEP(*(.vectors))
	} > FLASH
}
