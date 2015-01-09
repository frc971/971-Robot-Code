see ../README.txt for overall information

[FILES]
config/ has some configuration files
	aos.conf has directions for setting up resource limits so you can run the code on any linux machine (the directions are there to keep them with the file)
	setup_rc_caps.sh is a shell script (you have to run as root) that lets you run the realtime code without installing aos.conf for an individual file
	starter is an init.d file
		install it by putting it in /etc/init.d an running "update-rc.d starter defaults"
		restart it by running "invoke-rc.d starter restart" (doesn't always work very well...)
	the .config files are for building linux kernels

linux/ has code that only runs on linux systems

common/ is where stuff that runs on all platforms is
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
	would require explicit specification of volatile storage, as well as other implementation-defined
	restrictions.

The C++ namespace aos is used for all of the aos code. The crio and linux_code namespaces are for APIs that only make sense on one platform or the other.

Almost everything in aos/ is thread-safe. Files with a main() might not be, and some pieces of code explicitly say at their declaration that they aren't. Also, code which shares non-const pointers (including this) is not generally thread-safe. Other than those exceptions, everything is assumed to be thread-safe so the comments don't say so explicitly.
