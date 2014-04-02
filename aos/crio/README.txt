see ../README.txt for stuff affecting crio and linux code
There isn't much cRIO code left any more... At this point, we're basically
removing all of the #ifdeffed etc stuff in aos/common/ because the code is
nowhere close to compiling for the cRIO by now and it makes it harder to see
everything else.

[NOTES]
The assumption that sizeof(pointers) == sizeof(int) == sizeof(UINT32) == sizeof(uint32_t) == 4 is all over the crio code. The vxworks apis use UINT32 to pass user-defined arguments, and just passing pointers through those makes the code a lot simpler.
