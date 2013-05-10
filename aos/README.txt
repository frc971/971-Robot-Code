see ../README.txt for overall information

[FILES]
config/ has some configuration files
	aos.conf has directions for setting up resource limits so you can run the code on any linux machine (the directions are there to keep them with the file)
	setup_rc_caps.sh is a shell script (you have to run as root) that lets you run the realtime code without installing aos.conf for an individual file
	starter is an init.d file
		install it by putting it in /etc/init.d an running "update-rc.d starter defaults"
		restart it by running "invoke-rc.d starter restart" (doesn't always work very well...)
	the .config files are for building linux kernels

atom_code/ has code that only runs on the atom
crio/ has code that only runs on the crio

common/ is where stuff that runs on both the crio and the atom is
common/input/ is where the framework code for reading stuff into the queues system is
common/output/ is where the framework for writing things out of the queues system is
common/messages is where the c++ wrappers for "queues" are

[NOTES]
Some functions need to be in separate translation units in order for them to be guaranteed to work. As the C standard says,
	Alternatively, an implementation might perform various optimizations within each translation unit, such
	that the actual semantics would agree with the abstract semantics only when making function calls across
	translation unit boundaries. In such an implementation, at the time of each function entry and function
	return where the calling function and the called function are in different translation units, the values of all
	externally linked objects and of all objects accessible via pointers therein would agree with the abstract
	semantics. Furthermore, at the time of each such function entry the values of the parameters of the called
	function and of all objects accessible via pointers therein would agree with the abstract semantics. In this
	type of implementation, objects referred to by interrupt service routines activated by the signal function
	would require explicit speciﬁcation of volatile storage, as well as other implementation-deﬁned
	restrictions.
Everything that has to do different things when compiled for the crio or the atom uses #ifdef __VXWORKS__ etc.
The crio "queue" implementation uses 1 global instance and no synchronization, so it should only be used in code that gets called by the crio-side control loop infrastructure.
The C++ namespace aos is used for all of the aos code. The crio and atom namespaces are for APIs that only make sense on one platform or the other.
