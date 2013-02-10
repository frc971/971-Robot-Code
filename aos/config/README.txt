[FILES]

fitpc_kernel.config is a kernel configuration file for a fit-pc2
	it is currently for kernel version 3.2.21
	directions to use
		download the 3.2.21 vanilla kernel source and the 3.2.21 rt patch
		extract the kernel source and apply the patch
		if on a 64-bit x86 machine, create a 32-bit chroot to build in
		make sure fakeroot and kernel-package are installed
		in the linux-x.x.x directory: fakeroot make-kpkg --jobs=4 kernel_image

