package aos;

/**
 * Provides support for dealing with loading native code.
 */
public class NativeLoader {
	/**
	 * Loads a native library.
	 * @param name the name of the gyp shared_library or loadable_module target to load
	 */
	public static void load(String name) {
		System.load(System.getProperty("one-jar.expand.dir") + "/so_libs/lib" + name + ".so");
	}
}
