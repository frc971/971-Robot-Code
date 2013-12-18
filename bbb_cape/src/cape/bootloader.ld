MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 16k
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 128k
}

SECTIONS
{
	.text :
	{
		KEEP(*(.vectors))
	} > FLASH
}
