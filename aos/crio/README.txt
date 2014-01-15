see ../README.txt for stuff affecting crio and linux code
There isn't much cRIO code left any more... The general policy is to not break
things in aos/common/ that are #ifdeffed etc to with on the cRIO for no reason,
but if there's a major rewrite on anything the vxworks-specific code should
probably just be deleted. Also, any new stuff in aos/common/ may not work under
vxworks at all.

[NOTES]
The assumption that sizeof(pointers) == sizeof(int) == sizeof(UINT32) == sizeof(uint32_t) == 4 is all over the crio code. The vxworks apis use UINT32 to pass user-defined arguments, and just passing pointers through those makes the code a lot simpler.
