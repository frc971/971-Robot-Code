MEMORY
{
  FLASH (rx) : ORIGIN = 0x00008000, LENGTH = 0xF8000   /* 1024k - 32k */
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x20000   /* 128k */
}

SECTIONS
{
	.text :
	{
		KEEP(*(Reset_Handler))
	} > FLASH
}
