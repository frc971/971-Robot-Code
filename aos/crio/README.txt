see ../README.txt for stuff affecting crio and atom code
There isn't much cRIO code left any more...

[NOTES]
The assumption that sizeof(pointers) == sizeof(int) == sizeof(UINT32) == sizeof(uint32_t) == 4 is all over the crio code. The vxworks apis use UINT32 to pass user-defined arguments, and just passing pointers through those makes the code a lot simpler.
